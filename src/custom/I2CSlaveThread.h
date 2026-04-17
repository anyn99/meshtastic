#pragma once

#include "concurrency/OSThread.h"
#include <stdint.h>
#include <stddef.h>

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
    static constexpr size_t   REG_MAX     = 32; // max bytes per register

    /* EEPROM contents — index 0 = 0x50, index 1 = 0x51 */
    uint8_t eeprom[2][EEPROM_SIZE];

    /* Register device (0x40) — fill before enabling */
    uint8_t reg_data[REG_COUNT][REG_MAX];
    uint8_t reg_size[REG_COUNT]; // number of valid bytes per register

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
