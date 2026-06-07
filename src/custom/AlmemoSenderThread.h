#pragma once

#include "concurrency/OSThread.h"
#include "configuration.h"

#include "Almemo_I2C-Sensoren.h"
#include "AlmemoLed.h"
#include "AlmemoPacket.h"
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

class AlmemoSenderThread : public concurrency::OSThread
{
    enum SensorSource : uint8_t {
        SRC_NONE   = 0,
        SRC_ALMEMO = 1,
        SRC_SHT    = 2,
    };

    AlmemoI2CSensor almemo;
    Adafruit_SHT31  sht;

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

    /** ALMEMO @0x50 zuerst, dann SHT @0x44. */
    bool probeSensors()
    {
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

    /** Sensorquelle abfragen; unterscheidet abgesteckt (Disconnected) von Lesefehler (Error). */
    ReadStatus readSensor(float &temp, float &humi)
    {
        using RR = AlmemoI2CSensor::ReadResult;
        switch (source) {
        case SRC_ALMEMO: {
            RR tr = almemo.readValue(0, temp);
            RR hr = almemo.readValue(1, humi);
            if (tr != RR::Ok)
                temp = NAN;
            if (hr != RR::Ok)
                humi = NAN;
            /* Mindestens ein gültiger Wert → Send-Zyklus läuft. */
            if (tr == RR::Ok || hr == RR::Ok)
                return ReadStatus::Ok;
            /* Beide Slots ohne ACK → Sensor abgesteckt. Sonst (Gerät da, aber
             * status != 0x40) ein echter Lesefehler. */
            if (tr == RR::NotConnected && hr == RR::NotConnected)
                return ReadStatus::Disconnected;
            return ReadStatus::Error;
        }
        case SRC_SHT: {
            temp = sht.readTemperature();
            humi = sht.readHumidity();
            if (!isnan(temp) && !isnan(humi))
                return ReadStatus::Ok;
            /* SHT liefert NaN sowohl bei Abstecken als auch bei Busfehler →
             * als abgesteckt behandeln, damit neu geprobt wird. */
            return ReadStatus::Disconnected;
        }
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

        float t = NAN, h = NAN;
        ReadStatus rs = readSensor(t, h);
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

        AlmemoSensorPacket pkt;
        pkt.node_id   = nodeDB->getNodeNum();
        pkt.timestamp = getTime();
        pkt.temp      = t;
        pkt.humi      = h;

        static_assert(sizeof(AlmemoSensorPacket) <= sizeof(meshtastic_MeshPacket::decoded.payload.bytes),
                      "AlmemoSensorPacket too large for MeshPacket payload");

        meshtastic_MeshPacket *p = router->allocForSending();
        p->to                   = NODENUM_BROADCAST;
        p->channel              = ALMEMO_CHANNEL_INDEX;
        p->decoded.portnum      = meshtastic_PortNum_PRIVATE_APP;
        p->priority             = meshtastic_MeshPacket_Priority_DEFAULT;
        memcpy(p->decoded.payload.bytes, &pkt, sizeof(pkt));
        p->decoded.payload.size = sizeof(pkt);

        service->sendToMesh(p, RX_SRC_LOCAL);

        if (almemoLedThread) {
            almemoLedThread->setState(AlmemoLedState::Idle); // Fehlerzustand löschen
            almemoLedThread->pulseSend();                    // grüner Sende-Puls
        }

        LOG_DEBUG("AlmemoSender [%s] temp: %.2f  humi: %.2f",
                  source == SRC_ALMEMO ? "ALMEMO" : "SHT", pkt.temp, pkt.humi);

        if (sensorPowerSaving()) {
            waitingForTxToSleep = true;
            waitStartMs = millis();
            return ALMEMO_SENDER_THREADINTERVAL_MS;
        }
        return intervalMs();
    }
};
