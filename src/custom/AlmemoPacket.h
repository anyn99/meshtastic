#pragma once

#include <stdint.h>

/**
 * Binary payload for ALMEMO sensor packets (PRIVATE_APP portnum).
 * 16 bytes, little-endian floats.
 *
 * Shared between sender (EmulatorThread / SHT85 reader) and
 * receiver (AlmemoReceiverModule → I2CSlaveThread).
 */
struct __attribute__((packed)) AlmemoSensorPacket {
    uint32_t node_id;   /* Meshtastic NodeNum (last 4 bytes of BLE MAC) */
    uint32_t timestamp; /* Unix time in seconds (0 = time not yet known) */
    float    temp;      /* Temperature in °C */
    float    humi;      /* Relative humidity in %rH */
};
