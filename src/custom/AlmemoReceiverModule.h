#pragma once

#include "AlmemoPacket.h"
#include "concurrency/OSThread.h"
#include "mesh/SinglePortModule.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifndef ALMEMO_CHANNEL_INDEX
#define ALMEMO_CHANNEL_INDEX 1
#endif

/* Fallback timeout if a node has only sent one packet so far (no interval
 * estimate available yet). After this much silence the slot is cleared. */
#ifndef ALMEMO_RX_FIRST_PACKET_TIMEOUT_MS
#define ALMEMO_RX_FIRST_PACKET_TIMEOUT_MS 60000u
#endif

/* Duration to NACK all I2C transactions after a slot is cleared, so the
 * master rescans and updates its sensor count. */
#ifndef ALMEMO_RX_MUTE_MS
#define ALMEMO_RX_MUTE_MS 2000u
#endif

/**
 * I2CSlaveThread
 *
 * Runs a bitbang I2C slave on I2C_SDA / I2C_SCL (GPIOTE PORT events).
 * Responds on three addresses:
 *
 *   0x50, 0x51 — EEPROM emulation (24C02 style, 256 bytes each)
 *   0x40       — Register device (registers 0x00–0x03, one per sensor slot)
 *
 * Multi-value tracking
 * --------------------
 * Up to MAX_NODES (4) value streams can be represented simultaneously — the
 * hard cap of the emulated ALMEMO device (4 EEPROM info blocks + 4 registers).
 * Each stream is keyed by (node_id, sub), where sub is the sender's ALMEMO
 * slot index, so one sender can contribute several values (temp, humidity, …),
 * each landing in its own slot. The slot's Kommentar shows "NNNN.S"
 * (node short name + sub); unit and exponent come straight from the packet.
 *
 * onValue() is the entry point from the mesh (once per AlmemoValue): allocates
 * a slot for unseen (node_id, sub) pairs, updates timestamps and the live
 * value register for known ones.
 *
 * runOnce() expires slots whose sender went silent longer than its measured
 * inter-arrival interval × 1.05 (or ALMEMO_RX_FIRST_PACKET_TIMEOUT_MS if only
 * one packet has been received). On expiration the slot is reset to the empty
 * template and the I2C slave goes silent for ALMEMO_RX_MUTE_MS so the master
 * rescans.
 */
class I2CSlaveThread : public concurrency::OSThread
{
  public:
    static constexpr size_t   EEPROM_SIZE = 256;
    static constexpr uint8_t  REG_COUNT   = 4;
    static constexpr size_t   REG_MAX     = 4; // bytes per register (always 4)
    static constexpr uint8_t  MAX_NODES   = 4; // one slot per node (temp only)

    /* EEPROM contents — index 0 = 0x50, index 1 = 0x51 */
    uint8_t eeprom[2][EEPROM_SIZE];

    /* Register device (0x40) — reg[i] holds the latest temp for nodes[i] */
    uint8_t reg_data[REG_COUNT][REG_MAX];
    uint8_t reg_size[REG_COUNT]; // number of valid bytes per register (0..4)

    I2CSlaveThread();

    /**
     * Ingest one value from the mesh. Allocates a slot for new (node_id, sub)
     * pairs (populating the EEPROM info block from unit + exponent), refreshes
     * timestamps and updates the live value register for known ones.
     * Called from the Router task, once per AlmemoValue in a packet.
     *
     * @param raw  raw int16 reading; physical = raw * 10^exponent.
     */
    void onValue(uint32_t node_id, uint8_t sub, const char unit[2], int8_t exponent, int16_t raw);

    /**
     * Safely write into the virtual EEPROM from task context.
     * Disables the GPIOTE IRQ around the copy so the ISR never sees a torn buffer.
     */
    void writeEeprom(uint8_t addr, uint8_t offset, const uint8_t *data, uint8_t len);

    /**
     * Safely write 4 bytes into a register from task context.
     * Disables the GPIOTE IRQ around the copy so the ISR never sees a torn buffer.
     */
    void writeReg(uint8_t reg, const uint8_t data[REG_MAX]);

  protected:
    int32_t runOnce() override;

  private:
    struct NodeEntry {
        uint32_t node_id;
        uint8_t  sub;          /* sender ALMEMO slot index — part of the stream key */
        uint32_t last_seen_ms;
        uint32_t prev_seen_ms; /* 0 = only one packet seen so far — interval unknown */
        bool     in_use;
    };

    NodeEntry nodes[MAX_NODES] = {};
    uint32_t  muteUntilMs = 0;       /* 0 = not muted; otherwise wall-time deadline */

    uint8_t findOrAllocSlot(uint32_t node_id, uint8_t sub);
    void    initSlotForNode(uint8_t slot, uint32_t node_id, uint8_t sub, const char unit[2], int8_t exponent);
    void    clearSlot(uint8_t slot);
    void    writeRawForSlot(uint8_t slot, int16_t raw);
    void    expireSlot(uint8_t slot);
    void muteI2CFor(uint32_t ms);
    void unmuteI2C();

    /* Internal state (accessed from ISR — keep volatile) */
    static volatile uint8_t s_eeprom_ptr[2]; // current address pointer per EEPROM
    static volatile uint8_t s_reg_ptr;       // current register for 0x40

    /* Singleton pointer so static callbacks can reach instance data */
    static I2CSlaveThread *s_instance;

    /* Write notification — unused on the read-only EEPROM path (kept for #if 0) */
    static volatile bool    s_write_pending;
    static volatile uint8_t s_write_addr;
    static volatile uint8_t s_write_start;
    static volatile uint8_t s_write_count;

    static void     onWrite(uint8_t addr, const uint8_t *buf, uint8_t len);
    static uint8_t  onRead (uint8_t addr, uint8_t *buf, uint8_t max_len);
};

/**
 * AlmemoReceiverModule
 *
 * Listens for AlmemoSensorPacket messages on PRIVATE_APP /
 * ALMEMO_CHANNEL_INDEX and forwards them into the I2CSlaveThread for
 * per-node slot management.
 */
class AlmemoReceiverModule : public SinglePortModule
{
    I2CSlaveThread *slave;

  public:
    explicit AlmemoReceiverModule(I2CSlaveThread *s)
        : SinglePortModule("AlmemoRx", meshtastic_PortNum_PRIVATE_APP), slave(s) {}

  protected:
    bool wantPacket(const meshtastic_MeshPacket *p) override
    {
        return p->decoded.portnum == ourPortNum && p->channel == ALMEMO_CHANNEL_INDEX;
    }

    ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override
    {
        const size_t size = mp.decoded.payload.size;
        if (size < offsetof(AlmemoSensorPacket, values)) {
            LOG_WARN("AlmemoRx: runt payload %u", (unsigned)size);
            return ProcessMessage::CONTINUE;
        }

        AlmemoSensorPacket pkt;
        memcpy(&pkt, mp.decoded.payload.bytes, size < sizeof(pkt) ? size : sizeof(pkt));

        if (pkt.version != ALMEMO_PACKET_VERSION) {
            LOG_WARN("AlmemoRx: version %u != %u (sender out of date?)", pkt.version, ALMEMO_PACKET_VERSION);
            return ProcessMessage::CONTINUE;
        }
        if (pkt.count > ALMEMO_MAX_VALUES || size != almemoPacketSize(pkt.count)) {
            LOG_WARN("AlmemoRx: bad packet (count=%u size=%u)", pkt.count, (unsigned)size);
            return ProcessMessage::CONTINUE;
        }

        for (uint8_t i = 0; i < pkt.count; i++) {
            const AlmemoValue &v = pkt.values[i];
            slave->onValue(pkt.node_id, v.slot, v.unit, v.exponent, v.raw);
        }

        LOG_INFO("AlmemoRx: node=%08x t=%lu count=%u",
                 pkt.node_id, (unsigned long)pkt.timestamp, pkt.count);

        return ProcessMessage::CONTINUE;
    }
};
