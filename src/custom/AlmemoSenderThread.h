#pragma once

#include "concurrency/OSThread.h"
#include "configuration.h"

#include "Almemo_I2C-Sensoren.h"
#include "AlmemoLed.h"
#include "AlmemoPacket.h"
#if defined(ALMEMO_EINK)
#include "AlmemoEinkDisplay.h"
#endif
#include "Default.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "NodeStatus.h"
#include "Router.h"
#include "gps/RTC.h"
#include "power/PowerHAL.h"
#include "sleep.h"
#include <Adafruit_SHT31.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

#ifdef NRF52_SERIES
#include <FreeRTOS.h>
#include <task.h>
extern "C" {
#include "nrf_sdm.h"
}
#endif

/**
 * AlmemoSenderThread
 *
 * Sucht beim Start nach einer Sensorquelle am I2C-Bus (zuerst ALMEMO @0x50,
 * dann SHT31/85 @0x44) und broadcastet Temperatur + Feuchte als
 * AlmemoSensorPacket auf Mesh-Kanal ALMEMO_CHANNEL_INDEX (Default: 1) via
 * PRIVATE_APP-Portnum.
 *
 * Solange kein Sensor erkannt wurde, wird alle 500 ms erneut geprobt.
 */

#ifndef ALMEMO_CHANNEL_INDEX
#define ALMEMO_CHANNEL_INDEX 1
#endif

#ifndef ALMEMO_SENDER_DEFAULT_INTERVAL_SECS
#define ALMEMO_SENDER_DEFAULT_INTERVAL_SECS 30
#endif

#ifndef ALMEMO_SENDER_THREADINTERVAL_MS
#define ALMEMO_SENDER_THREADINTERVAL_MS 50
#endif

#ifndef ALMEMO_SENDER_PROBE_INTERVAL_MS
#define ALMEMO_SENDER_PROBE_INTERVAL_MS 500
#endif

#ifndef ALMEMO_SENDER_TX_WAIT_TIMEOUT_MS
#define ALMEMO_SENDER_TX_WAIT_TIMEOUT_MS 30000
#endif

#ifndef ALMEMO_SENDER_MIN_SLEEP_MS
#define ALMEMO_SENDER_MIN_SLEEP_MS 100
#endif

#ifndef ALMEMO_SHT_ADDR
#define ALMEMO_SHT_ADDR 0x44
#endif

#ifndef ALMEMO_TCA_ADDR
#define ALMEMO_TCA_ADDR 0x70 // I2C-Multiplexer (TCA9548A)
#endif

class AlmemoSenderThread : public concurrency::OSThread
{
    enum SensorSource : uint8_t {
        SRC_NONE        = 0,
        SRC_ALMEMO      = 1,
        SRC_SHT         = 2,
        SRC_ALMEMO_MULTI = 3, // mehrere ALMEMO-Sensoren hinter Mux @0x70
    };

    static constexpr uint8_t TCA_CHANNELS = 8; // TCA9548A: 8 schaltbare Kanäle

    AlmemoI2CSensor almemo;
    Adafruit_SHT31  sht;

    /* SRC_ALMEMO_MULTI: ein ALMEMO-Treiber pro Mux-Kanal, der einen Sensor trägt. */
    AlmemoI2CSensor almemoMux[TCA_CHANNELS];
    bool            muxPresent[TCA_CHANNELS] = {};
    uint8_t         muxCount = 0;

    /** Ergebnis eines Sensor-Lesezyklus über alle Quellen hinweg. */
    enum class ReadStatus : uint8_t {
        Ok,           // mindestens ein gültiger Wert
        Disconnected, // Sensor nicht mehr erreichbar → zurück auf SRC_NONE / gelb
        Error,        // Quelle antwortet, liefert aber keinen gültigen Wert → rot
    };

    SensorSource source = SRC_NONE;
    bool waitingForTxToSleep = false;
    uint32_t waitStartMs = 0;

    static bool sensorPowerSaving()
    {
        if (powerHAL_isVBUSConnected()) // USB steckt: wach bleiben, damit das Gerät erreichbar ist
            return false;
        return config.device.role == meshtastic_Config_DeviceConfig_Role_SENSOR && config.power.is_power_saving;
    }

    static uint32_t intervalMs()
    {
        return Default::getConfiguredOrDefaultMsScaled(moduleConfig.telemetry.environment_update_interval,
                                                       ALMEMO_SENDER_DEFAULT_INTERVAL_SECS, nodeStatus->getNumOnline());
    }

    static const char *sourceName(SensorSource s)
    {
        switch (s) {
        case SRC_ALMEMO:       return "ALMEMO";
        case SRC_ALMEMO_MULTI: return "ALMEMO_MULTI";
        case SRC_SHT:          return "SHT";
        default:               return "NONE";
        }
    }

    /** Einen der 8 Mux-Kanäle durchschalten (1 Bit pro Kanal); >7 wird ignoriert. */
    static void tcaSelect(uint8_t ch)
    {
        if (ch >= TCA_CHANNELS)
            return;
        Wire.beginTransmission(ALMEMO_TCA_ADDR);
        Wire.write(1 << ch);
        Wire.endTransmission();
    }

    /** Alle Mux-Kanäle abschalten (kein Sensor durchgereicht). */
    static void tcaDeselect()
    {
        Wire.beginTransmission(ALMEMO_TCA_ADDR);
        Wire.write(0x00);
        Wire.endTransmission();
    }

    /** Reines Adress-Probing des Multiplexers @0x70. */
    static bool tcaPresent()
    {
        Wire.beginTransmission(ALMEMO_TCA_ADDR);
        return Wire.endTransmission() == 0;
    }

    /**
     * Mux @0x70 erkennen und alle 8 Kanäle nach ALMEMO-Sensoren durchproben.
     * Pro Kanal genau einen Sensor (es ist immer nur ein Kanal durchgeschaltet).
     * Gefundene Sensoren werden in almemoMux[]/muxPresent[] gemerkt.
     * @return true wenn der Mux da ist und mindestens ein Sensor gefunden wurde.
     */
    bool probeMux()
    {
        if (!tcaPresent())
            return false;

        muxCount = 0;
        for (uint8_t ch = 0; ch < TCA_CHANNELS; ch++) {
            tcaSelect(ch);
            muxPresent[ch] = almemoMux[ch].begin(Wire);
            if (muxPresent[ch]) {
                muxCount++;
                LOG_INFO("AlmemoSender: mux ch%u ALMEMO ready (%u slots)", ch, almemoMux[ch].numPresent());
            }
        }
        tcaDeselect();

        if (muxCount == 0) {
            LOG_DEBUG("AlmemoSender: mux @0x%02x present but no ALMEMO on any channel", ALMEMO_TCA_ADDR);
            return false;
        }
        LOG_INFO("AlmemoSender: mux @0x%02x ready (%u sensor(s))", ALMEMO_TCA_ADDR, muxCount);
        return true;
    }

    /** Mux @0x70 zuerst, dann ALMEMO @0x50, dann SHT @0x44. */
    bool probeSensors()
    {
        if (probeMux()) {
            source = SRC_ALMEMO_MULTI;
            return true;
        }
        if (almemo.begin(Wire)) {
            source = SRC_ALMEMO;
            LOG_INFO("AlmemoSender: ALMEMO source ready (%u slots)", almemo.numPresent());
            return true;
        }
        if (sht.begin(ALMEMO_SHT_ADDR)) {
            source = SRC_SHT;
            LOG_INFO("AlmemoSender: SHT @0x%02x ready", ALMEMO_SHT_ADDR);
            return true;
        }
        return false;
    }

    /** Einen AlmemoValue an pkt anhängen, sofern noch Platz ist. */
    static void appendValue(AlmemoSensorPacket &pkt, uint8_t slotId, char u0, char u1, int8_t exp, int16_t raw)
    {
        if (pkt.count >= ALMEMO_MAX_VALUES)
            return;
        AlmemoValue &v = pkt.values[pkt.count++];
        v.slot     = slotId;
        v.unit[0]  = u0;
        v.unit[1]  = u1;
        v.exponent = exp;
        v.raw      = raw;
    }

    /**
     * Alle vorhandenen Slots eines ALMEMO-Geräts roh auslesen und als
     * AlmemoValue (Einheit + Exponent + Rohwert) an pkt anhängen.
     * @param slotIdBase Offset auf die Slot-Identität (Mux später: kanal<<2).
     */
    static ReadStatus appendAlmemoValues(AlmemoI2CSensor &dev, AlmemoSensorPacket &pkt, uint8_t slotIdBase)
    {
        using RR = AlmemoI2CSensor::ReadResult;
        bool anyOk = false, anyAnswered = false;
        for (uint8_t s = 0; s < AlmemoI2CSensor::MAX_SLOTS; s++) {
            const AlmemoI2CSensor::SlotInfo &info = dev.slot(s);
            if (!info.present)
                continue;
            int16_t raw;
            RR rr = dev.readRaw(s, raw);
            if (rr == RR::Ok) {
                appendValue(pkt, slotIdBase + s, info.unit[0], info.unit[1], info.exponent, raw);
                anyOk = anyAnswered = true;
            } else if (rr == RR::BadValue) {
                anyAnswered = true; /* Gerät antwortet, Wert ungültig */
            }
            /* NotConnected: dieser Slot ackt nicht (mehr) */
        }
        if (anyOk)
            return ReadStatus::Ok;
        /* Kein gültiger Wert: Gerät noch da (BadValue) → Fehler; sonst abgesteckt. */
        return anyAnswered ? ReadStatus::Error : ReadStatus::Disconnected;
    }

    /** SHT als 2 native ALMEMO-Werte (°C, %H, Exponent -2) an pkt anhängen. */
    ReadStatus appendShtValues(AlmemoSensorPacket &pkt)
    {
        float t = sht.readTemperature();
        float h = sht.readHumidity();
        /* SHT liefert NaN sowohl bei Abstecken als auch bei Busfehler →
         * als abgesteckt behandeln, damit neu geprobt wird. */
        if (isnan(t) && isnan(h))
            return ReadStatus::Disconnected;
        if (!isnan(t))
            appendValue(pkt, 0, (char)0xF8, 'C', -2, (int16_t)lroundf(t * 100.0f));
        if (!isnan(h))
            appendValue(pkt, 1, '%', 'H', -2, (int16_t)lroundf(h * 100.0f));
        return ReadStatus::Ok;
    }

    /** Paket aus der aktiven Quelle befüllen; klassifiziert für die LED-Logik. */
    ReadStatus buildValues(AlmemoSensorPacket &pkt)
    {
        pkt.count = 0;
        switch (source) {
        case SRC_ALMEMO:
            return appendAlmemoValues(almemo, pkt, 0);
        case SRC_ALMEMO_MULTI: {
            /* Interim: nur das erste gefundene Mux-Gerät wird gesendet.
             * Vor dem Zugriff den passenden Kanal durchschalten. */
            for (uint8_t ch = 0; ch < TCA_CHANNELS; ch++) {
                if (!muxPresent[ch])
                    continue;
                tcaSelect(ch);
                return appendAlmemoValues(almemoMux[ch], pkt, 0);
            }
            return ReadStatus::Disconnected;
        }
        case SRC_SHT:
            return appendShtValues(pkt);
        default:
            return ReadStatus::Disconnected;
        }
    }

  public:
    AlmemoSenderThread() : OSThread("AlmemoSender") {}

  protected:
    int32_t runOnce() override
    {
        if (waitingForTxToSleep) {
            if (almemoLedThread)
                almemoLedThread->pulseSend(); // Grün halten, solange noch gesendet wird
            bool txDone = doPreflightSleep();
            bool timedOut = (millis() - waitStartMs) > ALMEMO_SENDER_TX_WAIT_TIMEOUT_MS;
            if (!txDone && !timedOut)
                return ALMEMO_SENDER_THREADINTERVAL_MS;

            waitingForTxToSleep = false;
            uint32_t cycleMs = intervalMs();
            uint32_t elapsed = millis();
            uint32_t sleepMs = (elapsed < cycleMs) ? (cycleMs - elapsed) : ALMEMO_SENDER_MIN_SLEEP_MS;
            LOG_DEBUG("AlmemoSender: deep sleep %ums (cycle=%u, awake=%u%s)", sleepMs, cycleMs, elapsed,
                      timedOut ? ", tx timeout" : "");
            if (almemoLedThread)
                almemoLedThread->off(); // LED vor Deep Sleep aus
            doDeepSleep(sleepMs, true, true);
            return ALMEMO_SENDER_THREADINTERVAL_MS; // unreached
        }

        if (source == SRC_NONE) {
            if (!probeSensors()) {
                if (almemoLedThread)
                    almemoLedThread->setState(AlmemoLedState::NoSensor); // gelb blinken
                return ALMEMO_SENDER_PROBE_INTERVAL_MS;
            }
        }

        AlmemoSensorPacket pkt;
        pkt.version   = ALMEMO_PACKET_VERSION;
        pkt.node_id   = nodeDB->getNodeNum();
        pkt.timestamp = getTime();

        ReadStatus rs = buildValues(pkt);
        if (rs != ReadStatus::Ok) {
            if (rs == ReadStatus::Disconnected) {
                LOG_WARN("AlmemoSender: sensor disconnected, re-probing");
                source = SRC_NONE; // zurück auf Anfang → erneutes Probing
                if (almemoLedThread)
                    almemoLedThread->setState(AlmemoLedState::NoSensor); // gelb blinken
            } else {
                LOG_WARN("AlmemoSender: sensor read error (invalid value)");
                if (almemoLedThread)
                    almemoLedThread->setState(AlmemoLedState::Error); // rot blinken
            }
            return ALMEMO_SENDER_PROBE_INTERVAL_MS;
        }

        static_assert(sizeof(AlmemoSensorPacket) <= sizeof(meshtastic_MeshPacket::decoded.payload.bytes),
                      "AlmemoSensorPacket too large for MeshPacket payload");

        size_t wireSize = almemoPacketSize(pkt.count);

        meshtastic_MeshPacket *p = router->allocForSending();
        p->to                   = NODENUM_BROADCAST;
        p->channel              = ALMEMO_CHANNEL_INDEX;
        p->decoded.portnum      = meshtastic_PortNum_PRIVATE_APP;
        p->priority             = meshtastic_MeshPacket_Priority_DEFAULT;
        memcpy(p->decoded.payload.bytes, &pkt, wireSize);
        p->decoded.payload.size = wireSize;

        service->sendToMesh(p, RX_SRC_LOCAL);

        if (almemoLedThread) {
            almemoLedThread->setState(AlmemoLedState::Idle); // Fehlerzustand löschen
            almemoLedThread->pulseSend();                    // grüner Sende-Puls
        }

        LOG_DEBUG("AlmemoSender [%s] %u value(s) sent", sourceName(source), pkt.count);

#if defined(ALMEMO_EINK)
        // E-Paper mit aktuellem Sendeintervall + Timestamp des gerade gesendeten Pakets versorgen
        almemoEinkPublish(intervalMs() / 1000, pkt.timestamp);
#endif

        if (sensorPowerSaving()) {
            waitingForTxToSleep = true;
            waitStartMs = millis();
            return ALMEMO_SENDER_THREADINTERVAL_MS;
        }
        return intervalMs();
    }
};
