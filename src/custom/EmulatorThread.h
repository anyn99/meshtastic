#pragma once

#include "concurrency/OSThread.h"
#include "configuration.h"
#include <stdlib.h>

#include "AlmemoPacket.h"

#if defined(ALMEMO_SENSOR_RECEIVER)
#include "AlmemoReceiverModule.h"
#else
#include "MeshService.h"
#include "NodeDB.h"
#include "Router.h"
#include "gps/RTC.h"
#include <string.h>
#endif

/**
 * EmulatorThread
 *
 * Generates synthetic sensor readings (temperature + humidity) for testing.
 *
 * Broadcasts an AlmemoSensorPacket on mesh channel ALMEMO_CHANNEL_INDEX
 * (default: 1) via PRIVATE_APP portnum.
 *
 * Receiver mode (ALMEMO_SENSOR_RECEIVER, slave != nullptr):
 *   Also writes the readings into the I2CSlaveThread register device (0x40)
 *   so the ALMEMO master can read them via I2C.
 *
 *   reg 0: temperature  — 4-byte format [0x00 0x40 MSB LSB], fixed-point *100
 *   reg 1: humidity     — same format
 *
 * Interval: hijacks moduleConfig.detection_sensor.state_broadcast_secs so the
 * rate is editable from the phone app. 0 → EMULATOR_DEFAULT_INTERVAL_MS.
 */

#ifndef ALMEMO_CHANNEL_INDEX
#define ALMEMO_CHANNEL_INDEX 1
#endif

#ifndef EMULATOR_DEFAULT_INTERVAL_MS
#define EMULATOR_DEFAULT_INTERVAL_MS 1500
#endif

class EmulatorThread : public concurrency::OSThread
{
#if defined(ALMEMO_SENSOR_RECEIVER)
    I2CSlaveThread *slave;
#endif

  public:
#if defined(ALMEMO_SENSOR_RECEIVER)
    explicit EmulatorThread(I2CSlaveThread *s = nullptr) : OSThread("Emulator"), slave(s) {}
#else
    EmulatorThread() : OSThread("Emulator") {}
#endif

  protected:
    int32_t runOnce() override
    {
        int16_t temp = 2400 + (rand() % 101) - 50;
        int16_t humi = 5000 + (rand() % 1001) - 500;

#if defined(ALMEMO_SENSOR_RECEIVER)
        /* --- I2C slave registers ----------------------------------------- */
        if (slave) {
            uint8_t datatemp[I2CSlaveThread::REG_MAX] = {
                0x00,
                0x40,
                (uint8_t)((uint16_t)temp >> 8),
                (uint8_t)((uint16_t)temp & 0xFF),
            };
            slave->writeReg(0, datatemp);

            uint8_t datahumi[I2CSlaveThread::REG_MAX] = {
                0x00,
                0x40,
                (uint8_t)((uint16_t)humi >> 8),
                (uint8_t)((uint16_t)humi & 0xFF),
            };
            slave->writeReg(1, datahumi);
        }
#else
        /* --- Mesh packet (sender only) ----------------------------------- */
        AlmemoSensorPacket pkt;
        pkt.node_id   = nodeDB->getNodeNum();
        pkt.timestamp = getTime();
        pkt.temp      = temp / 100.0f;
        pkt.humi      = humi / 100.0f;

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
#endif

        uint32_t intervalSecs = 0; //moduleConfig.detection_sensor.state_broadcast_secs;
        int32_t  nextMs       = (intervalSecs > 0) ? (int32_t)(intervalSecs * 1000) : EMULATOR_DEFAULT_INTERVAL_MS;

        LOG_DEBUG("Emulator temp: %d.%02d degC  humi: %d.%02d %%rH  next: %d ms",
                  temp / 100, temp % 100, humi / 100, humi % 100, nextMs);

        return nextMs;
    }
};
