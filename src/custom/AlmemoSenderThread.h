#pragma once

#include "concurrency/OSThread.h"
#include "configuration.h"

#include "AlmemoPacket.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "Router.h"
#include "gps/RTC.h"
#include <Adafruit_SHT31.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

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

#ifndef ALMEMO_SENDER_INTERVAL_MS
#define ALMEMO_SENDER_INTERVAL_MS 1500
#endif

#ifndef ALMEMO_SHT_ADDR
#define ALMEMO_SHT_ADDR 0x44
#endif

class AlmemoSenderThread : public concurrency::OSThread
{
    Adafruit_SHT31 sht;
    bool sensorReady = false;

  public:
    AlmemoSenderThread() : OSThread("AlmemoSender")
    {
        sensorReady = sht.begin(ALMEMO_SHT_ADDR);
        LOG_INFO("AlmemoSender: SHT @0x%02x %s", ALMEMO_SHT_ADDR, sensorReady ? "ready" : "begin() failed");
    }

  protected:
    int32_t runOnce() override
    {
        if (!sensorReady) {
            sensorReady = sht.begin(ALMEMO_SHT_ADDR);
            if (!sensorReady)
                return ALMEMO_SENDER_INTERVAL_MS;
            LOG_INFO("AlmemoSender: SHT @0x%02x ready (retry)", ALMEMO_SHT_ADDR);
        }

        float t = sht.readTemperature();
        float h = sht.readHumidity();
        if (isnan(t) || isnan(h)) {
            LOG_WARN("AlmemoSender: SHT read NaN, skip");
            return ALMEMO_SENDER_INTERVAL_MS;
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

        LOG_DEBUG("AlmemoSender temp: %.2f degC  humi: %.2f %%rH", t, h);
        return ALMEMO_SENDER_INTERVAL_MS;
    }
};
