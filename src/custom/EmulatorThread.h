#pragma once

#include "I2CSlaveThread.h"
#include "concurrency/OSThread.h"
#include "configuration.h"
#include <stdlib.h>

/**
 * EmulatorThread
 *
 * Generates synthetic sensor readings (temperature + humidity) for testing.
 *
 * Receiver mode (slave != nullptr):
 *   Writes the readings into the I2CSlaveThread register device (0x40) every
 *   second so the ALMEMO master can read them via I2C.
 *
 * Sender / standalone mode (slave == nullptr):
 *   Just logs the values — useful for testing without the I2C slave stack.
 *
 * Register layout (4 bytes each):
 *   [0] 0x00  — status high  (0x00 0x40 = value valid & current)
 *   [1] 0x40  — status low
 *   [2] value MSB  (16-bit fixed-point, 2 decimal places)
 *   [3] value LSB
 *
 *   reg 0: temperature near 24.00 °C (±0.50 °C random walk)
 *   reg 1: relative humidity near 50.00 %rH (±5.00 % random walk)
 */
class EmulatorThread : public concurrency::OSThread
{
    I2CSlaveThread *slave;

  public:
    explicit EmulatorThread(I2CSlaveThread *s = nullptr) : OSThread("Emulator"), slave(s) {}

  protected:
    int32_t runOnce() override
    {
        int16_t temp = 2400 + (rand() % 101) - 50;
        int16_t humi = 5000 + (rand() % 1001) - 500;

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

        LOG_DEBUG("Emulator temp: %d.%02d degC", temp / 100, temp % 100);
        LOG_DEBUG("Emulator humi: %d.%02d %%rH", humi / 100, humi % 100);

        return 1000; /* next run in 1 s */
    }
};
