#pragma once

#include <stddef.h>
#include <stdint.h>

/**
 * AlmemoCommon — gemeinsame ALMEMO-Kernschicht.
 *
 * Enthält alles, was sich Sender, Empfänger und die einzelnen Sensortreiber
 * (I2C-EEPROM, D7-UART, …) teilen, und NICHTS Hardware-Spezifisches:
 *   - das Mesh-Wire-Format (AlmemoValue / AlmemoSensorPacket),
 *   - das Slot-Byte-Packing (channel/valueIndex),
 *   - die Mess-Dimension eines Slots (AlmemoSlot: Einheit + Exponent) samt
 *     Skalierung zwischen Rohwert und physikalischem Wert,
 *   - das Paket-Bau-Primitiv almemoAppendValue().
 *
 * Die Treiber lesen nur Hardware und liefern AlmemoSlot-Metadaten + Rohwerte;
 * der Sender setzt daraus über almemoAppendValue() das Paket zusammen.
 */

// =========================================================================
//  Wire-Format (Sender ⟷ Empfänger, PRIVATE_APP portnum)
// =========================================================================

#ifndef ALMEMO_MAX_VALUES
/* Total value entries carried per packet, across all channels/slots. The slot
 * byte (see below) allows up to 16 channels * 16 value slots, but a single
 * packet rarely needs that many. Each entry is 6 bytes on the wire, so 16 ->
 * 106-byte packet, well within the mesh payload limit. Bump for denser setups. */
#define ALMEMO_MAX_VALUES 16
#endif

/* Bump whenever the wire layout changes; receiver rejects mismatched senders.
 *   v2 -> v3: slot byte split changed from 2/6 (valueIndex/channel) to 4/4, so a
 *             single device can carry up to 16 value slots (e.g. an ALMEMO D7
 *             with up to 10 Messstellen). */
#define ALMEMO_PACKET_VERSION 3

/**
 * One typed measurement from a single ALMEMO slot. The packet mirrors the
 * sensor's native model: raw int16 + power-of-ten exponent + 2-byte unit, so the
 * receiver relays them 1:1 into its emulated EEPROM (no fixed scaling assumed).
 *
 *   physical value = raw * 10^exponent
 */
struct __attribute__((packed)) AlmemoValue {
    uint8_t slot;     /* packed identity within the sender, (channel << 4) | valueIndex:
                       *   bits 0-3 : valueIndex — which measured value of the sensor (0..15)
                       *   bits 4-7 : channel    — which sensor (0..15). channel 0 means the
                       *              node sends loose single values (e.g. an SHT, no ALMEMO
                       *              sensor attached); ALMEMO sensors are numbered from 1. */
    char    unit[2];  /* ALMEMO unit bytes (NOT null-terminated) */
    int8_t  exponent; /* signed power-of-ten exponent */
    int16_t raw;      /* raw sensor reading; physical = raw * 10^exponent */
};

/* Variable length on the wire: only `count` entries are sent, not the full
 * values[] array — use almemoPacketSize(count) when sending and when validating. */
struct __attribute__((packed)) AlmemoSensorPacket {
    uint8_t     version;   /* = ALMEMO_PACKET_VERSION */
    uint32_t    node_id;   /* Meshtastic NodeNum (last 4 bytes of BLE MAC) */
    uint32_t    timestamp; /* Unix time in seconds (0 = time not yet known) */
    uint8_t     count;     /* number of valid entries in values[] (0..ALMEMO_MAX_VALUES) */
    AlmemoValue values[ALMEMO_MAX_VALUES];
};

// =========================================================================
//  Messwert-Slot, wie ihn ein Treiber liefert (von beiden Treibern direkt genutzt)
// =========================================================================

/**
 * Ein Messwert-Slot eines ALMEMO-Geräts, so wie ihn ein Treiber liefert:
 * seine Position (→ Paket-valueIndex) plus die Mess-Dimension (Einheit +
 * Zehnerpotenz-Exponent). Das ist alles, was man braucht, um aus einem später
 * gelesenen Rohwert einen AlmemoValue zu bauen — beide Treiber (I2C, D7)
 * benutzen diesen Typ direkt, ganz ohne eigene SlotInfo-Struktur.
 *
 *   physikalischer Wert = raw * 10^exponent
 */
struct AlmemoSlot {
    uint8_t valueIndex; /* Position im Gerät → wird im Paket der valueIndex.
                         *   I2C-EEPROM-Sensor: Slot-Index 0..3
                         *   D7-UART-Sensor:    Messstellen-Nummer 0..9 */
    char    unit[3];    /* 2 Einheit-Bytes + NUL (° als 0xF8) */
    int8_t  exponent;   /* signierter Zehnerpotenz-Exponent */
};

// =========================================================================
//  Funktionen (Bodies in AlmemoCommon.cpp)
// =========================================================================

/** On-wire size carrying only `count` value entries. */
size_t almemoPacketSize(uint8_t count);

/* Pack/unpack the AlmemoValue::slot byte — the ONE place that knows the 4/4 split
 * (bits 4-7 channel, bits 0-3 valueIndex). Sender and receiver both go through here. */
uint8_t almemoSlotId(uint8_t channel, uint8_t valueIndex);
uint8_t almemoSlotChannel(uint8_t slot);
uint8_t almemoSlotValueIndex(uint8_t slot);

/** 10^e für e in [-4..+4] (digitale ALMEMO-Auflösung), sonst 1.0. */
float almemoPow10f(int8_t e);
/** Rohwert → physikalisch: value = raw * 10^exponent. */
float almemoScaleFromRaw(int16_t raw, int8_t exponent);
/** Physikalisch → Rohwert: raw = value * 10^(-exponent), auf int16 begrenzt. */
int16_t almemoScaleToRaw(float value, int8_t exponent);

/**
 * Einen AlmemoValue an pkt anhängen, sofern noch Platz ist (count < MAX).
 * @return true wenn angehängt, false wenn das Paket voll war.
 */
bool almemoAppendValue(AlmemoSensorPacket &pkt, uint8_t slotId, char u0, char u1, int8_t exponent, int16_t raw);
