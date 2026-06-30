#pragma once

#include "concurrency/OSThread.h"
#include "configuration.h"

#include "Almemo_I2C-Sensoren.h"
#include "AlmemoD7Sensor.h"
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
#include <stdlib.h>
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

#if defined(ALMEMO_EINK)
// Persistenter Sendezähler über Warmstarts hinweg. Der getimte Deep-Sleep ist in Wahrheit
// delay()+NVIC_SystemReset() (ein Warmstart, bei dem das SRAM erhalten bleibt); nur ein
// echter Power-Cycle löscht es. Die Variable liegt in .noinit (vom Startup NICHT genullt,
// siehe nrf52840_s140_v7.ld), ein Magic schützt gegen kalt gestartetes (zufälliges) RAM.
static constexpr uint32_t ALMEMO_SEND_COUNT_MAGIC = 0x414c4d43; // "ALMC"
struct AlmemoSendCount {
    uint32_t magic;
    uint32_t count;
    uint32_t check; // magic ^ count ^ 0xA5A5A5A5
};
__attribute__((section(".noinit"))) static AlmemoSendCount almemoSendCount;

// Zähler um 1 erhöhen und zurückgeben; bei kaltem/korruptem RAM bei 1 beginnen.
static uint32_t almemoNextSendCount()
{
    if (almemoSendCount.magic != ALMEMO_SEND_COUNT_MAGIC ||
        almemoSendCount.check != (ALMEMO_SEND_COUNT_MAGIC ^ almemoSendCount.count ^ 0xA5A5A5A5u))
        almemoSendCount.count = 0;
    almemoSendCount.count++;
    almemoSendCount.magic = ALMEMO_SEND_COUNT_MAGIC;
    almemoSendCount.check = ALMEMO_SEND_COUNT_MAGIC ^ almemoSendCount.count ^ 0xA5A5A5A5u;
    return almemoSendCount.count;
}
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
        SRC_EMULATOR    = 4,  // -DALMEMO_EMULATOR: randomisierte Testwerte statt echtem I2C
        SRC_ALMEMO_D7   = 5,  // -DALMEMO_SENSOR_D7: I2C-Erkennung (0x7B), Werte über UART
    };

    static constexpr uint8_t TCA_CHANNELS = 8; // TCA9548A: 8 schaltbare Kanäle

    AlmemoI2CSensor almemo;
    Adafruit_SHT31  sht;
    AlmemoD7Sensor  d7; // SRC_ALMEMO_D7: I2C-Erkennung + UART-Messwerte

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
        case SRC_EMULATOR:     return "EMU";
        case SRC_ALMEMO_D7:    return "ALMEMO_D7";
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

    /**
     * Belegung der Mux-Kanäle erneut prüfen (Plug/Unplug einzelner Kanäle).
     * Neu erkannte Kanäle werden per begin() eingelesen, weggefallene markiert.
     * Wird beim Senden aufgerufen (kein separates Polling).
     */
    void refreshMux()
    {
        if (!tcaPresent()) { // ganzer Mux weg
            for (uint8_t ch = 0; ch < TCA_CHANNELS; ch++)
                muxPresent[ch] = false;
            muxCount = 0;
            return;
        }
        muxCount = 0;
        for (uint8_t ch = 0; ch < TCA_CHANNELS; ch++) {
            tcaSelect(ch);
            const bool present = AlmemoI2CSensor::probe(Wire);
            if (present && !muxPresent[ch])
                muxPresent[ch] = almemoMux[ch].begin(Wire); // neu angesteckt -> Metadaten laden
            else if (!present)
                muxPresent[ch] = false; // abgesteckt
            if (muxPresent[ch])
                muxCount++;
        }
        tcaDeselect();
    }

    /** Mux @0x70 zuerst, dann ALMEMO @0x50, dann SHT @0x44. */
    bool probeSensors()
    {
#if defined(ALMEMO_EMULATOR)
        source = SRC_EMULATOR; // kein echtes I2C-Probing; Quelle ist immer "vorhanden"
        return true;
#elif defined(ALMEMO_SENSOR_D7)
        /* D7: Erkennung allein über das Typ-Byte 0x7B (I2C-Handshake). Danach
         * laufen die Messwerte über UART. Entweder ein D7 wird erkannt – oder
         * eben nicht. */
        if (AlmemoD7Sensor::probe(Wire)) {
            /* Erkannt → UART öffnen und einmalig die Setup-Sequenz fahren. */
            Serial1.begin(AlmemoD7Sensor::UART_BAUD);
            d7.beginUart(Serial1);
            source = SRC_ALMEMO_D7;
            LOG_INFO("AlmemoSender: ALMEMO D7 ready");
            return true;
        }
        return false;
#else
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
#endif
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
     * @param slotIdBase channel<<2: oberer Teil der slot-Identität. Der ALMEMO-
     *        Sensor-Channel startet bei 1 (0 ist Einzelwerten wie SHT vorbehalten),
     *        valueIndex 0..3 (= Slot s im Gerät) wird hier addiert.
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

    /**
     * Randomisierte Testwerte statt echtem Sensor (-DALMEMO_EMULATOR): Temp
     * ~24 °C ±0,5 und Feuchte ~50 % ±5 als 2 native ALMEMO-Werte (Exponent -2,
     * °C / %H, channel 0 = Einzelwerte). Läuft im normalen Sender-Takt.
     */
    static ReadStatus appendEmulatorValues(AlmemoSensorPacket &pkt)
    {
        int16_t temp = 2400 + (rand() % 101) - 50;
        int16_t humi = 5000 + (rand() % 1001) - 500;
        appendValue(pkt, 0, (char)0xF8, 'C', -2, temp);
        appendValue(pkt, 1, '%', 'H', -2, humi);
        LOG_DEBUG("AlmemoEmulator: temp=%d.%02d degC humi=%d.%02d %%rH", temp / 100, temp % 100, humi / 100, humi % 100);
        return ReadStatus::Ok;
    }

    /** D7-Messwert über UART lesen und als nativen ALMEMO-Wert (°C, Exp -2) anhängen. */
    ReadStatus appendD7Values(AlmemoSensorPacket &pkt)
    {
        float t;
        /* Keine Antwort innerhalb des Timeouts → Sensor abgesteckt → neu proben. */
        if (!d7.readValue(t))
            return ReadStatus::Disconnected;
        appendValue(pkt, 0, (char)0xF8, 'C', -2, (int16_t)lroundf(t * 100.0f));
        return ReadStatus::Ok;
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
            /* Einzelner ALMEMO-Sensor (kein Mux) → channel 1. */
            return appendAlmemoValues(almemo, pkt, 1u << 2);
        case SRC_ALMEMO_MULTI: {
            /* Alle vorhandenen Mux-Kanäle senden: vor jedem Gerät den passenden
             * Kanal durchschalten und dessen Werte anhängen. Mux-Kanal ch →
             * Paket-channel ch+1 (channel 0 = Einzelwerte/SHT). */
            bool anyOk = false, anyAnswered = false;
            for (uint8_t ch = 0; ch < TCA_CHANNELS; ch++) {
                if (!muxPresent[ch])
                    continue;
                tcaSelect(ch);
                ReadStatus rs = appendAlmemoValues(almemoMux[ch], pkt, (uint8_t)((ch + 1) << 2));
                if (rs == ReadStatus::Ok)
                    anyOk = anyAnswered = true;
                else if (rs == ReadStatus::Error)
                    anyAnswered = true;
                /* Disconnected: dieser Kanal liefert (gerade) nichts → überspringen */
            }
            return anyOk ? ReadStatus::Ok : (anyAnswered ? ReadStatus::Error : ReadStatus::Disconnected);
        }
        case SRC_SHT:
            return appendShtValues(pkt);
        case SRC_EMULATOR:
            return appendEmulatorValues(pkt);
        case SRC_ALMEMO_D7:
            return appendD7Values(pkt);
        default:
            return ReadStatus::Disconnected;
        }
    }

#if defined(ALMEMO_EINK)
    /** Name (max 8, getrimmt) + Wertanzahl in eine Display-Sensorinfo schreiben. */
    static void setEinkSensor(AlmemoEinkSensorInfo &e, const char *name, uint8_t count)
    {
        e.present    = true;
        e.valueCount = count;
        size_t n = 0;
        if (name)
            for (; n < 8 && name[n]; n++)
                e.name[n] = name[n];
        e.name[n] = '\0';
        while (n > 0 && e.name[n - 1] == ' ') // ALMEMO-Namen sind oft mit Leerzeichen aufgefüllt
            e.name[--n] = '\0';
    }

    /** Die 4 Display-Rechtecke (hinten 1/2, vorne 3/4) aus der aktiven Quelle befüllen. */
    void fillEinkSensors(AlmemoEinkSensorInfo out[4]) const
    {
        for (int i = 0; i < 4; i++)
            out[i] = AlmemoEinkSensorInfo{};
        switch (source) {
        case SRC_ALMEMO:
            setEinkSensor(out[0], almemo.deviceName(), almemo.numPresent());
            break;
        case SRC_SHT:
            setEinkSensor(out[0], "SHT", 2); // Temperatur + Feuchte
            break;
        case SRC_EMULATOR:
            setEinkSensor(out[0], "EMU", 2); // randomisierte Temp + Feuchte
            break;
        case SRC_ALMEMO_MULTI:
            // Mux-Kanal ch -> Rechteck ch (feste Position); nur die ersten 4 Kanäle.
            for (uint8_t ch = 0; ch < 4; ch++)
                if (muxPresent[ch])
                    setEinkSensor(out[ch], almemoMux[ch].deviceName(), almemoMux[ch].numPresent());
            break;
        default:
            break;
        }
    }

    // Sendezähler-Stand des letzten gesendeten Pakets (für Display-Updates ohne neues Senden).
    uint32_t lastSendCount = 0;

    /** E-Paper mit dem aktuellen Sensor-/Sendestand neu zeichnen. */
    void publishEinkNow(uint32_t sendCount)
    {
        AlmemoEinkSensorInfo s[4] = {};
        fillEinkSensors(s);
        almemoEinkPublish(intervalMs() / 1000, sendCount, s);
    }
#endif

  public:
    AlmemoSenderThread() : OSThread("AlmemoSender") {}

  protected:
    // Verbleibende Wartezeit bis zum nächsten Zyklus: Intervall minus bereits verstrichener
    // Zeit, min. ALMEMO_SENDER_MIN_SLEEP_MS. Beide Pfade teilen sich diese Formel; sie messen
    // die verstrichene Zeit nur unterschiedlich (siehe Aufrufstellen) – beides ms-genau über
    // millis(). Bewusst KEIN getTime(): das wäre nur sekundengenau. millis() startet nach dem
    // Warmstart bei 0 und ist damit pro Boot die präzise Referenz für die Wachzeit.
    static uint32_t remainingCycleMs(uint32_t elapsedMs)
    {
        const uint32_t iv = intervalMs();
        return (elapsedMs < iv) ? (iv - elapsedMs) : ALMEMO_SENDER_MIN_SLEEP_MS;
    }

    int32_t runOnce() override
    {
        const uint32_t cycleStartMs = millis();

        if (waitingForTxToSleep) {
            if (almemoLedThread)
                almemoLedThread->pulseSend(); // Grün halten, solange noch gesendet wird
            bool txDone = doPreflightSleep();
            bool timedOut = (millis() - waitStartMs) > ALMEMO_SENDER_TX_WAIT_TIMEOUT_MS;
            if (!txDone && !timedOut)
                return ALMEMO_SENDER_THREADINTERVAL_MS;

            waitingForTxToSleep = false;
            // Der Sleep-Zyklus liegt komplett in EINEM Boot (Wake -> Senden -> Sleep) und
            // millis() ist nach dem Warmstart 0, daher ist millis() hier direkt die reine
            // Wachzeit dieses Zyklus.
            const uint32_t awakeMs = millis();
            const uint32_t sleepMs = remainingCycleMs(awakeMs);
            LOG_DEBUG("AlmemoSender: deep sleep %ums (cycle=%u, awake=%u%s)", sleepMs, intervalMs(), awakeMs,
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

        // Mux: einzelne Kanäle beim Senden neu prüfen (Plug/Unplug). Fällt alles weg,
        // wie Disconnect behandeln und das Display einmalig leeren.
        if (source == SRC_ALMEMO_MULTI) {
            refreshMux();
            if (muxCount == 0) {
                LOG_WARN("AlmemoSender: all mux sensors gone, re-probing");
                source = SRC_NONE;
                if (almemoLedThread)
                    almemoLedThread->setState(AlmemoLedState::NoSensor); // gelb blinken
#if defined(ALMEMO_EINK)
                publishEinkNow(lastSendCount); // leeres Display, Zähler unverändert
#endif
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
#if defined(ALMEMO_EINK)
                publishEinkNow(lastSendCount); // Sensor weg -> Display leeren, Zähler unverändert
#endif
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
        p->hop_limit            = 0; // nur direkte Nachbarn, kein Weiterleiten im Mesh
        memcpy(p->decoded.payload.bytes, &pkt, wireSize);
        p->decoded.payload.size = wireSize;

        service->sendToMesh(p, RX_SRC_LOCAL);

        if (almemoLedThread) {
            almemoLedThread->setState(AlmemoLedState::Idle); // Fehlerzustand löschen
            almemoLedThread->pulseSend();                    // grüner Sende-Puls
        }

        LOG_DEBUG("AlmemoSender [%s] %u value(s), %u bytes sent", sourceName(source), pkt.count, (unsigned)wireSize);

#if defined(ALMEMO_EINK)
        // E-Paper mit Intervall + Sendezähler + den 4 Sensor-Infos (Name/Wertanzahl) versorgen.
        // Der Zähler überlebt die Deep-Sleep-Warmstarts (.noinit) und wird nur hier hochgezählt.
        lastSendCount = almemoNextSendCount();
        publishEinkNow(lastSendCount);
#endif

        if (sensorPowerSaving()) {
            waitingForTxToSleep = true;
            waitStartMs = millis();
            return ALMEMO_SENDER_THREADINTERVAL_MS;
        }

        // Echten Sendeabstand halten: die Laufzeit dieses Zyklus (Sensor-Lesen, Senden,
        // E-Paper-Refresh ~2,6 s) vom Intervall abziehen – sonst wäre der Abstand
        // intervalMs() + Arbeitszeit. Kein Reboot hier, daher relativ zu cycleStartMs
        // (die Thread-Delay-Pause aus dem Vorzyklus liegt davor und zählt nicht mit).
        return remainingCycleMs(millis() - cycleStartMs);
    }
};
