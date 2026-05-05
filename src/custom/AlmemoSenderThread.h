#pragma once

#include "concurrency/OSThread.h"
#include "configuration.h"

#include "AlmemoPacket.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "Router.h"
#include "gps/RTC.h"
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

#ifdef HFCLK_DBG_PIN
extern "C" void hfclk_dbg_irq_dump(void);
#endif

/**
 * AlmemoSenderThread
 *
 * Reads a Sensirion SHT31/SHT85 (or compatible, 0x44) via the standard Wire
 * bus and broadcasts temp + humidity as an AlmemoSensorPacket on mesh channel
 * ALMEMO_CHANNEL_INDEX (default: 1) via PRIVATE_APP portnum.
 *
 * The sensor is expected to have been detected by the I2C scanner already
 * (Wire is initialized). If begin() fails at startup we keep retrying.
 */

#ifndef ALMEMO_CHANNEL_INDEX
#define ALMEMO_CHANNEL_INDEX 1
#endif

// Total cycle time: boot + sensor read + TX + sleep. Deep sleep duration is
// (ALMEMO_SENDER_INTERVAL_MS - millis() at sleep entry), so the cadence stays
// constant regardless of how long boot/TX takes.
#ifndef ALMEMO_SENDER_INTERVAL_MS
#define ALMEMO_SENDER_INTERVAL_MS 30000
#endif

// OSThread poll cadence — used between TX completion checks and as the retry
// interval when the sensor isn't ready yet. Must be << ALMEMO_SENDER_INTERVAL_MS.
#ifndef ALMEMO_SENDER_THREADINTERVAL_MS
#define ALMEMO_SENDER_THREADINTERVAL_MS 50
#endif

// Hard cap so we never get stuck in the wait-for-TX state if something jams the radio.
#ifndef ALMEMO_SENDER_TX_WAIT_TIMEOUT_MS
#define ALMEMO_SENDER_TX_WAIT_TIMEOUT_MS ALMEMO_SENDER_INTERVAL_MS
#endif

// Floor for the deep-sleep duration if boot+TX took longer than the cycle.
#ifndef ALMEMO_SENDER_MIN_SLEEP_MS
#define ALMEMO_SENDER_MIN_SLEEP_MS 100
#endif

#ifndef ALMEMO_SHT_ADDR
#define ALMEMO_SHT_ADDR 0x44
#endif

class AlmemoSenderThread : public concurrency::OSThread
{
    Adafruit_SHT31 sht;
    bool sensorReady = false;
    bool waitingForTx = false;
    uint32_t waitStartMs = 0;

    static bool sensorPowerSaving()
    {
        return config.device.role == meshtastic_Config_DeviceConfig_Role_SENSOR && config.power.is_power_saving;
    }

  public:
    AlmemoSenderThread() : OSThread("AlmemoSender")
    {
        sensorReady = sht.begin(ALMEMO_SHT_ADDR);
        LOG_INFO("AlmemoSender: SHT @0x%02x %s", ALMEMO_SHT_ADDR, sensorReady ? "ready" : "begin() failed");
    }

    void dumpPowerDiag()
    {
#ifdef NRF52_SERIES
        uint8_t sdEn = 0;
        sd_softdevice_is_enabled(&sdEn);
        LOG_INFO("PowerDiag: SoftDevice enabled=%u", sdEn);

        // HFCLKSTAT bit 0 = SRC (0=HFINT/RC 16MHz, 1=HFXO crystal),
        // bit 16 = STATE (0=stopped, 1=running). HFXO running ≈ +700 µA.
        uint32_t hfStat = NRF_CLOCK->HFCLKSTAT;
        uint32_t hfRun = NRF_CLOCK->HFCLKRUN;
        LOG_INFO("PowerDiag: HFCLKSTAT=0x%08x (src=%s, state=%s) HFCLKRUN=0x%08x", (unsigned)hfStat,
                 (hfStat & CLOCK_HFCLKSTAT_SRC_Msk) ? "HFXO" : "HFINT",
                 (hfStat & CLOCK_HFCLKSTAT_STATE_Msk) ? "RUN" : "STOP", (unsigned)hfRun);
        LOG_INFO("PowerDiag: USBD ENABLE=%u USBPULLUP=%u USBREGSTATUS=0x%08x DCDCEN=%u", (unsigned)NRF_USBD->ENABLE,
                 (unsigned)NRF_USBD->USBPULLUP, (unsigned)NRF_POWER->USBREGSTATUS, (unsigned)NRF_POWER->DCDCEN);

#ifdef HFCLK_DBG_PIN
        hfclk_dbg_irq_dump();
#endif

        // ~600 bytes for ~10-15 tasks; enlarge if your build creates more.
        // RedirectablePrint::printBuf is 160 B on nRF52, so a single multi-line
        // LOG_INFO(buf) gets truncated. Emit one log line per task instead.
        static char buf[1024];
        vTaskList(buf);
        LOG_INFO("PowerDiag: FreeRTOS tasks (Name State Prio Stack# Num):");
        for (char *line = buf, *next; line && *line; line = next) {
            next = strchr(line, '\n');
            if (next) {
                *next = '\0';
                next++;
            }
            size_t len = strlen(line);
            if (len > 0 && line[len - 1] == '\r')
                line[len - 1] = '\0';
            if (line[0])
                LOG_INFO("  %s", line);
        }

        // OSThreads run cooperatively inside the 'loop' FreeRTOS task and do not
        // appear in vTaskList. List them here with their next-run delay — short
        // intervals are the prime suspects for keeping the main loop awake (and
        // for triggering peripherals that implicitly request HFXO).
        unsigned long now = millis();
        int n = concurrency::mainController.size();
        LOG_INFO("PowerDiag: OSThreads in mainController (n=%d):", n);
        for (int i = 0; i < n; i++) {
            Thread *t = concurrency::mainController.get(i);
            if (!t)
                continue;
            LOG_INFO("  [%2d] %-20s en=%u tillRun=%ld", i, t->ThreadName.c_str(), t->enabled, t->tillRun(now));
        }
#endif
    }

  protected:
    int32_t runOnce() override
    {
        static bool diagDumped = false;
        if (!diagDumped) {
            diagDumped = true;
            dumpPowerDiag();
        }

        if (waitingForTx) {
            bool txDone = doPreflightSleep();
            bool timedOut = (millis() - waitStartMs) > ALMEMO_SENDER_TX_WAIT_TIMEOUT_MS;
            if (!txDone && !timedOut)
                return ALMEMO_SENDER_THREADINTERVAL_MS;

            waitingForTx = false;
            uint32_t elapsed = millis();
            uint32_t sleepMs = (elapsed < ALMEMO_SENDER_INTERVAL_MS) ? (ALMEMO_SENDER_INTERVAL_MS - elapsed)
                                                                     : ALMEMO_SENDER_MIN_SLEEP_MS;
            LOG_DEBUG("AlmemoSender: deep sleep %ums (cycle=%u, awake=%u%s)", sleepMs, ALMEMO_SENDER_INTERVAL_MS, elapsed,
                      timedOut ? ", tx timeout" : "");
            // skipPreflight=true: we already polled doPreflightSleep ourselves across runOnce calls,
            // so waitEnterSleep's blocking poll-loop (which would deadlock the radio thread) is bypassed.
            // skipSaveNodeDb=true: sender state is identical across cycles; skipping the multi-file
            // proto write saves ~1-2s of awake time (and flash wear) per cycle.
            doDeepSleep(sleepMs, true, true);
            return ALMEMO_SENDER_THREADINTERVAL_MS; // unreached
        }

        if (!sensorReady) {
            sensorReady = sht.begin(ALMEMO_SHT_ADDR);
            if (!sensorReady)
                return ALMEMO_SENDER_THREADINTERVAL_MS;
            LOG_INFO("AlmemoSender: SHT @0x%02x ready (retry)", ALMEMO_SHT_ADDR);
        }

        float t = sht.readTemperature();
        float h = sht.readHumidity();
        if (isnan(t) || isnan(h)) {
            LOG_WARN("AlmemoSender: SHT read NaN, skip");
            return ALMEMO_SENDER_THREADINTERVAL_MS;
        }

        AlmemoSensorPacket pkt;
        pkt.node_id = nodeDB->getNodeNum();
        pkt.timestamp = getTime();
        pkt.temp = t;
        pkt.humi = h;

        static_assert(sizeof(AlmemoSensorPacket) <= sizeof(meshtastic_MeshPacket::decoded.payload.bytes),
                      "AlmemoSensorPacket too large for MeshPacket payload");

        meshtastic_MeshPacket *p = router->allocForSending();
        p->to = NODENUM_BROADCAST;
        p->channel = ALMEMO_CHANNEL_INDEX;
        p->decoded.portnum = meshtastic_PortNum_PRIVATE_APP;
        p->priority = meshtastic_MeshPacket_Priority_DEFAULT;
        memcpy(p->decoded.payload.bytes, &pkt, sizeof(pkt));
        p->decoded.payload.size = sizeof(pkt);

        service->sendToMesh(p, RX_SRC_LOCAL);

        LOG_DEBUG("AlmemoSender temp: %.2f degC  humi: %.2f %%rH", pkt.temp, pkt.humi);

        if (sensorPowerSaving()) {
            waitingForTx = true;
            waitStartMs = millis();
            return ALMEMO_SENDER_THREADINTERVAL_MS;
        }
        return ALMEMO_SENDER_INTERVAL_MS;
    }
};
