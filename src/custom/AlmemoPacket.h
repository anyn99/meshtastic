#pragma once

#include <stddef.h>
#include <stdint.h>

/**
 * Binary payload for ALMEMO sensor packets (PRIVATE_APP portnum).
 *
 * Shared between sender (AlmemoSenderThread / EmulatorThread) and
 * receiver (AlmemoReceiverModule -> I2CSlaveThread).
 *
 * The packet mirrors the ALMEMO sensor's native data model: per measured
 * value we carry the raw int16 reading, its power-of-ten exponent and the
 * 2-byte unit, exactly as the sensor exposes them over I2C. The receiver
 * relays these 1:1 into its emulated EEPROM info block + value register, so
 * no fixed scaling (e.g. *100) is assumed anywhere.
 *
 *   physical value = raw * 10^exponent
 *
 * Wire format is variable length: only `count` AlmemoValue entries are sent,
 * not the full values[] array. Use almemoPacketSize(count) for the on-wire
 * size both when sending and when validating on receive.
 */

#ifndef ALMEMO_MAX_VALUES
#define ALMEMO_MAX_VALUES 8 /* single ALMEMO device has <=4 slots; headroom for mux later */
#endif

/* Bump whenever the wire layout changes; receiver rejects mismatched senders. */
#define ALMEMO_PACKET_VERSION 2

/** One typed measurement from a single ALMEMO slot. */
struct __attribute__((packed)) AlmemoValue {
    uint8_t slot;     /* identity within the sender: ALMEMO slot 0..3
                       * (mux later encodes the channel, e.g. channel<<2 | slot) */
    char    unit[2];  /* ALMEMO unit bytes (NOT null-terminated) */
    int8_t  exponent; /* signed power-of-ten exponent */
    int16_t raw;      /* raw sensor reading; physical = raw * 10^exponent */
};

struct __attribute__((packed)) AlmemoSensorPacket {
    uint8_t     version;   /* = ALMEMO_PACKET_VERSION */
    uint32_t    node_id;   /* Meshtastic NodeNum (last 4 bytes of BLE MAC) */
    uint32_t    timestamp; /* Unix time in seconds (0 = time not yet known) */
    uint8_t     count;     /* number of valid entries in values[] (0..ALMEMO_MAX_VALUES) */
    AlmemoValue values[ALMEMO_MAX_VALUES];
};

/** On-wire size carrying only `count` value entries. */
static inline size_t almemoPacketSize(uint8_t count)
{
    return offsetof(AlmemoSensorPacket, values) + (size_t)count * sizeof(AlmemoValue);
}
