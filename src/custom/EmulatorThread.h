#pragma once

#include "concurrency/OSThread.h"
#include "configuration.h"
#include <stdlib.h>

#include "AlmemoReceiverModule.h"
#include "Default.h"

/**
 * EmulatorThread (Receiver-Test)
 *
 * Schreibt synthetische Messwerte (Temperatur + Feuchte) direkt in die
 * I2C-Slave-Register (0x40) des I2CSlaveThread, damit der ALMEMO-Master ohne
 * realen Sender getestet werden kann.
 *
 *   reg 0: Temperatur  — 4-Byte [0x00 0x40 MSB LSB], Festkomma *100
 *   reg 1: Feuchte     — gleiches Format
 *
 * Der Sender-seitige Emulator (Mesh) steckt seit der Konsolidierung im
 * AlmemoSenderThread (aktiv via -DALMEMO_EMULATOR auf der Sender-Variante).
 */

#ifndef EMULATOR_DEFAULT_INTERVAL_MS
#define EMULATOR_DEFAULT_INTERVAL_MS 1500
#endif

class EmulatorThread : public concurrency::OSThread
{
    I2CSlaveThread *slave;

    // Sende-Takt in ms, zur Laufzeit über moduleConfig.telemetry.environment_update_interval (sek) konfigurierbar.
    static uint32_t intervalMs()
    {
        return Default::getConfiguredOrDefaultMsScaled(moduleConfig.telemetry.environment_update_interval,
                                                       EMULATOR_DEFAULT_INTERVAL_MS, nodeStatus->getNumOnline());
    }

  public:
    explicit EmulatorThread(I2CSlaveThread *s = nullptr) : OSThread("Emulator"), slave(s) {}

  protected:
    int32_t runOnce() override
    {
        int16_t temp = 2400 + (rand() % 101) - 50;
        int16_t humi = 5000 + (rand() % 1001) - 500;

        if (slave) {
            uint8_t datatemp[I2CSlaveThread::REG_MAX] = {
                0x00, 0x40, (uint8_t)((uint16_t)temp >> 8), (uint8_t)((uint16_t)temp & 0xFF),
            };
            slave->writeReg(0, datatemp);

            uint8_t datahumi[I2CSlaveThread::REG_MAX] = {
                0x00, 0x40, (uint8_t)((uint16_t)humi >> 8), (uint8_t)((uint16_t)humi & 0xFF),
            };
            slave->writeReg(1, datahumi);
        }

        uint32_t nextMs = intervalMs();
        if (nextMs == 0)
            nextMs = EMULATOR_DEFAULT_INTERVAL_MS;
        LOG_DEBUG("AlmemoEmulator(rx):  temp: %d.%02d degC  humi: %d.%02d %%rH  next: %d ms",
                  temp / 100, temp % 100, humi / 100, humi % 100, nextMs);

        return nextMs;
    }
};
