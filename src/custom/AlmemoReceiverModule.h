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

/**
 * I2CSlaveThread
 *
 * Runs a bitbang I2C slave on I2C_SDA / I2C_SCL (GPIOTE PORT events).
 * Responds on three addresses:
 *
 *   0x50, 0x51 — EEPROM emulation (24C02 style, 256 bytes each)
 *                Write: [mem_addr, data...]  sets pointer + writes
 *                Write: [mem_addr]           sets pointer only
 *                Read:                       returns bytes from pointer, auto-increment
 *
 *   0x40       — Register device (registers 0x00–0x03)
 *                Write: [reg_addr]           selects register
 *                Read:                       returns reg_data[reg_addr][0..reg_size[reg_addr]-1]
 *
 * Usage:
 *   Fill reg_data / reg_size before or after construction.
 *   Read/write eeprom[] directly for EEPROM contents.
 *   All fields accessed from ISR — keep accesses atomic or use disable/enable.
 */
class I2CSlaveThread : public concurrency::OSThread
{
  public:
    static constexpr size_t   EEPROM_SIZE = 256;
    static constexpr uint8_t  REG_COUNT   = 4;
    static constexpr size_t   REG_MAX     = 4; // bytes per register (always 4)

    /* EEPROM contents — index 0 = 0x50, index 1 = 0x51 */
    uint8_t eeprom[2][EEPROM_SIZE];

    /* Register device (0x40) — fill before enabling */
    uint8_t reg_data[REG_COUNT][REG_MAX];
    uint8_t reg_size[REG_COUNT]; // number of valid bytes per register (0..4)

    I2CSlaveThread();

    /**
     * Safely write into the virtual EEPROM from task context.
     * Disables the GPIOTE IRQ around the copy so the ISR never sees a torn buffer.
     *
     * @param addr    I2C address: 0x50 or 0x51
     * @param offset  Byte offset within the 256-byte EEPROM (0x00–0xFF)
     * @param data    Source data
     * @param len     Number of bytes to write (clamped to fit)
     */
    void writeEeprom(uint8_t addr, uint8_t offset, const uint8_t *data, uint8_t len);

    /**
     * Safely write 4 bytes into a register from task context.
     * Disables the GPIOTE IRQ around the copy so the ISR never sees a torn buffer.
     *
     * @param reg   Register index (0..REG_COUNT-1)
     * @param data  Exactly 4 bytes to write
     */
    void writeReg(uint8_t reg, const uint8_t data[REG_MAX]);

  protected:
    int32_t runOnce() override;

  private:
    /* Internal state (accessed from ISR — keep volatile) */
    static volatile uint8_t s_eeprom_ptr[2]; // current address pointer per EEPROM
    static volatile uint8_t s_reg_ptr;       // current register for 0x40

    /* Singleton pointer so static callbacks can reach instance data */
    static I2CSlaveThread *s_instance;

    /* Write notification — set from ISR, consumed in runOnce */
    static volatile bool    s_write_pending;
    static volatile uint8_t s_write_addr;   /* I2C address that was written */
    static volatile uint8_t s_write_start;  /* EEPROM pointer before the write */
    static volatile uint8_t s_write_count;  /* number of data bytes written    */

    static void     onWrite(uint8_t addr, const uint8_t *buf, uint8_t len);
    static uint8_t  onRead (uint8_t addr, uint8_t *buf, uint8_t max_len);
};

/**
 * AlmemoReceiverModule
 *
 * Listens for AlmemoSensorPacket messages on PRIVATE_APP /
 * ALMEMO_CHANNEL_INDEX and forwards decoded sensor values into the
 * I2CSlaveThread register device (0x40):
 *
 *   reg 0: temperature  [0x00 0x40 MSB LSB]  fixed-point *100
 *   reg 1: humidity     [0x00 0x40 MSB LSB]  fixed-point *100
 *
 * Self-registers in the global MeshModule list on construction — no
 * further wiring needed beyond calling new AlmemoReceiverModule(slave).
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
        if (mp.decoded.payload.size != sizeof(AlmemoSensorPacket)) {
            LOG_WARN("AlmemoRx: unexpected payload size %u (expected %u)",
                     mp.decoded.payload.size, (unsigned)sizeof(AlmemoSensorPacket));
            return ProcessMessage::CONTINUE;
        }

        AlmemoSensorPacket pkt;
        memcpy(&pkt, mp.decoded.payload.bytes, sizeof(pkt));

        int16_t temp = (int16_t)(pkt.temp * 100.0f);
        int16_t humi = (int16_t)(pkt.humi * 100.0f);

        uint8_t datatemp[I2CSlaveThread::REG_MAX] = {
            0x00, 0x40,
            (uint8_t)((uint16_t)temp >> 8),
            (uint8_t)((uint16_t)temp & 0xFF),
        };
        slave->writeReg(0, datatemp);

        uint8_t datahumi[I2CSlaveThread::REG_MAX] = {
            0x00, 0x40,
            (uint8_t)((uint16_t)humi >> 8),
            (uint8_t)((uint16_t)humi & 0xFF),
        };
        slave->writeReg(1, datahumi);

        LOG_INFO("AlmemoRx: node=%08x t=%lu temp=%.2f humi=%.2f",
                 pkt.node_id, (unsigned long)pkt.timestamp, pkt.temp, pkt.humi);

        return ProcessMessage::CONTINUE;
    }
};
