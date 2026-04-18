#if defined(ALMEMO_SENSOR_RECEIVER)
#include "AlmemoReceiverModule.h"
#include "configuration.h"

extern "C" {
#include "i2c_bb_slave.h"
}

#include <nrf.h>      /* NRF_GPIOTE */
#include <string.h>

/* gpiote_dispatch.cpp owns attachInterrupt/detachInterrupt and the EVENTS_IN
 * callback table.  We call gpiote_dispatch_events_in() at the bottom of the
 * GPIOTE_IRQHandler so that both PORT events (I2C slave) and IN events (radio
 * DIO1, EXT_CHRG_DETECT, …) are serviced in the single ISR we own. */
void gpiote_dispatch_events_in();

/* -------------------------------------------------------------------------
 * loopCanSleep() override
 *
 * Without this override, main-nrf52.cpp returns !Serial — i.e. the main task
 * sleeps whenever USB CDC is not connected.  That triggers FreeRTOS tickless
 * idle (vPortSuppressTicksAndSleep), which calls sd_nvic_critical_region_enter
 * before and after sd_app_evt_wait().  sd_nvic_critical_region_enter raises
 * BASEPRI to configMAX_SYSCALL_INTERRUPT_PRIORITY (= 2), masking our GPIOTE
 * ISR (also priority 2) for the entire post-wakeup critical section (~5–15 µs).
 *
 * At 100 kHz I2C the SCL half-period is only 5 µs.  While BASEPRI=2 the master
 * drives one or more SCL edges; the ISR fires only after BASEPRI drops back to
 * 0.  By then scl_read() returns the current (already-changed) pin level, not
 * the level at LATCH time, so the state machine dispatches rising/falling edges
 * backwards and the I2C protocol breaks entirely.
 *
 * Returning false keeps the main task always runnable → idle task never
 * accumulates a long expected-idle-time → vPortSuppressTicksAndSleep is never
 * called → GPIOTE at priority 2 is never masked for more than a few hundred ns.
 *
 * This definition wins over main-nrf52.cpp's via project-object link order and
 * -Wl,--allow-multiple-definition.
 * ---------------------------------------------------------------------- */
bool loopCanSleep()
{
    return false;
}

/* -------------------------------------------------------------------------
 * Sensor definitions
 *
 * Mirrors the HASP ESP32 reference implementation.
 * SensorTyp 0x37 = digital sensor, 2-decimal fixed-point.
 * SensorTyp 0xFF = slot empty / not configured.
 *
 * EEPROM 0 (address 0x50) layout:
 *   0x00–0x07  device name      ("FHAD46  ")
 *   0x08–0x43  sensor slot 0    (DIGITAL_SENSOR_INFO_SIZE = 60 bytes)
 *   0x44–0x7F  sensor slot 1    (60 bytes)
 *   0x80–0xBB  sensor slot 2    (60 bytes)
 *   0xBC–0xF7  sensor slot 3    (60 bytes)
 *   0xF8–0xFF  firmware version ("   6.66")
 * ---------------------------------------------------------------------- */

#define DIGITAL_SENSOR_INFO_SIZE 60u

struct SensorChannelInfo {
    uint8_t SensorTyp;
    char    Einheit[3];    /* 2 printable chars + null */
    char    Kommentar[11]; /* 10 printable chars + null */
};

static const SensorChannelInfo s_sensors[] = {
    { 0xFF, { 0x00, 0x00, 0x00 }, { 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
    { 0x37, { '\xF8', 'C',  0x00 }, "Temp      " },
    { 0x37, { '%',    'H',  0x00 }, "RelHumidty" },
    { 0x37, { '%',    'C',  0x00 }, "CO2-Konz  " },
    { 0x37, { 'p',    'H',  0x00 }, "pH-Wert   " },
    { 0x37, { 'u',    'S',  0x00 }, "Leitf     " },
    { 0x37, { 'l',    'x',  0x00 }, "Licht     " },
};

/**
 * Fill a DIGITAL_SENSOR_INFO_SIZE-byte sensor info block in the virtual EEPROM.
 *
 * Buffer layout for SensorTyp 0x37:
 *   [0]     SensorTyp  (0x37)
 *   [1]     exponent byte: (8|2)<<4 = 0xA0 → 2 decimal places, negative exponent
 *   [7]     bitmask 0xFF
 *   [30-31] Einheit (2 bytes)
 *   [32-41] Kommentar (10 bytes)
 *   [59]    block length (60)
 *
 * For SensorTyp 0xFF only [0] is set; all other bytes are zeroed.
 */
static bool initSensorBuffer(const SensorChannelInfo *sensor, uint8_t *buf, size_t bufLen)
{
    memset(buf, 0, bufLen);
    if (bufLen < DIGITAL_SENSOR_INFO_SIZE)
        return false;

    if (sensor->SensorTyp == 0x37) {
        buf[0]  = sensor->SensorTyp;
        buf[1]  = (0x8 | 0x2) << 4; /* 0xA0 */
        buf[7]  = 0xFF;
        memcpy(&buf[30], sensor->Einheit,    2);
        memcpy(&buf[32], sensor->Kommentar, 10);
        buf[59] = DIGITAL_SENSOR_INFO_SIZE;
        return true;
    }

    if (sensor->SensorTyp == 0xFF) {
        buf[0] = sensor->SensorTyp;
        return true;
    }

    return false;
}

/* -------------------------------------------------------------------------
 * GPIOTE_IRQHandler — direct strong definition, overrides startup.S weak alias.
 *
 * --wrap=GPIOTE_IRQHandler does NOT work here: startup.S defines GPIOTE_IRQHandler
 * as a weak alias (defined, not undefined), so ld's --wrap only intercepts
 * undefined references and misses the vector table entry.
 *
 * Instead we define GPIOTE_IRQHandler directly as a strong C symbol.
 * I2C_BB_OWN_IRQ prevents i2c_bb_slave.c from also defining it.
 * WInterrupts.c (Arduino core) also defines it.  gpiote_dispatch.cpp provides
 * its own attachInterrupt/detachInterrupt that win over WInterrupts.c's versions
 * (project .o objects precede framework archives in the link order), so this
 * handler can call gpiote_dispatch_events_in() to dispatch EVENTS_IN callbacks
 * (radio DIO1, EXT_CHRG_DETECT, …) in addition to the PORT-event I2C path.
 * ---------------------------------------------------------------------- */

extern "C" void GPIOTE_IRQHandler(void)
{
    /* Debug: LED blue (Arduino pin 12 = P0.06 = native nRF pin 6) on while inside ISR.
     * LED stays on  → stuck inside ISR (hang/storm).
     * LED blinks once, system hangs → hang is outside ISR (task/scheduler). */
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0, 6)); /* ON — active-low, common anode */

    if (NRF_GPIOTE->EVENTS_PORT)
    {
        i2c_bb_slave_gpiote_irq_handler();
    }

    /* Dispatch EVENTS_IN callbacks (radio DIO1, EXT_CHRG_DETECT, …).
     * Clears each EVENTS_IN register before invoking the callback — prevents
     * interrupt storm if the callback does not de-assert the pin. */
    gpiote_dispatch_events_in();

    #if __CORTEX_M == 0x04
      __DSB(); __NOP();__NOP();__NOP();__NOP();
    #endif

    nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(0, 6)); /* LED OFF */

}

/* -------------------------------------------------------------------------
 * writeEeprom — ISR-safe EEPROM write from task context
 * ---------------------------------------------------------------------- */

void I2CSlaveThread::writeEeprom(uint8_t addr, uint8_t offset, const uint8_t *data, uint8_t len)
{
    if (!len)
        return;
    uint8_t idx = (addr == 0x51) ? 1 : 0;
    if (offset >= EEPROM_SIZE)
        return;
    if (len > EEPROM_SIZE - offset)
        len = EEPROM_SIZE - offset;

    NVIC_DisableIRQ(GPIOTE_IRQn);
    memcpy(&eeprom[idx][offset], data, len);
    NVIC_EnableIRQ(GPIOTE_IRQn);
}

/* -------------------------------------------------------------------------
 * writeReg — ISR-safe register write from task context
 * ---------------------------------------------------------------------- */

void I2CSlaveThread::writeReg(uint8_t reg, const uint8_t data[REG_MAX])
{
    if (reg >= REG_COUNT)
        return;

    NVIC_DisableIRQ(GPIOTE_IRQn);
    memcpy(reg_data[reg], data, REG_MAX);
    reg_size[reg] = REG_MAX;
    NVIC_EnableIRQ(GPIOTE_IRQn);
}

/* -------------------------------------------------------------------------
 * Static member definitions
 * ---------------------------------------------------------------------- */

volatile uint8_t  I2CSlaveThread::s_eeprom_ptr[2]  = {0, 0};
volatile uint8_t  I2CSlaveThread::s_reg_ptr         = 0;
I2CSlaveThread   *I2CSlaveThread::s_instance        = nullptr;
volatile bool     I2CSlaveThread::s_write_pending   = false;
volatile uint8_t  I2CSlaveThread::s_write_addr      = 0;
volatile uint8_t  I2CSlaveThread::s_write_start     = 0;
volatile uint8_t  I2CSlaveThread::s_write_count     = 0;

/* -------------------------------------------------------------------------
 * Constructor
 * ---------------------------------------------------------------------- */

I2CSlaveThread::I2CSlaveThread() : OSThread("I2CSlave")
{
    s_instance = this;

    memset(eeprom,   0, sizeof(eeprom));
    memset(reg_data, 0, sizeof(reg_data));
    memset(reg_size, 0, sizeof(reg_size));

    /* Populate EEPROM 0 (0x50) with device identity and sensor layout.
     * All four sensor slots default to "empty" (SensorTyp 0xFF = s_sensors[0]).
     * The master can later overwrite individual slots via I2C writes. */
    memcpy(&eeprom[0][0x00], "FHAD46  ", 8);
    strncpy((char*)&eeprom[0][0xF8], "   6.66", 8);
    initSensorBuffer(&s_sensors[1], &eeprom[0][0x08], DIGITAL_SENSOR_INFO_SIZE);
    initSensorBuffer(&s_sensors[2], &eeprom[0][0x44], DIGITAL_SENSOR_INFO_SIZE);
    initSensorBuffer(&s_sensors[0], &eeprom[0][0x80], DIGITAL_SENSOR_INFO_SIZE);
    initSensorBuffer(&s_sensors[0], &eeprom[0][0xBC], DIGITAL_SENSOR_INFO_SIZE);

    /* Init GPIO, arm PORT SENSE, enable GPIOTE interrupt. */
    i2c_bb_slave_init();

    /* Prepare blue LED (P0.06) for ISR debug (LED_STATE_ON=0, common anode). */
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 6));
    nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(0, 6)); /* start OFF */

    i2c_bb_slave_register(0x50, onWrite, onRead);
    i2c_bb_slave_register(0x51, onWrite, onRead);
    i2c_bb_slave_register(0x40, onWrite, onRead);

}

/* -------------------------------------------------------------------------
 * OSThread
 * ---------------------------------------------------------------------- */

int32_t I2CSlaveThread::runOnce()
{
    if (s_write_pending) {
        s_write_pending = false;
        uint8_t        idx   = (s_write_addr == 0x51) ? 1 : 0;
        uint8_t        start = s_write_start;
        uint8_t        count = s_write_count;
        const uint8_t *d     = eeprom[idx] + start;
        char           hex[3 * 32 + 1]; /* "xx " per byte, null-terminated */
        char          *hp    = hex;
        for (uint8_t i = 0; i < count; i++) {
            hp += snprintf(hp, 4, "%02x ", d[i]);
        }
        LOG_INFO("I2CSlave write 0x%02x @0x%02x (%u B): %s", s_write_addr, start, s_write_count, hex);
        return 0; /* sofort wiederkommen falls noch ein Write anliegt */
    }

    const volatile uint32_t *p = i2c_bb_slave_stats.scl_periods;
    uint8_t n = i2c_bb_slave_stats.scl_period_idx;
    if (n > I2C_BB_SCL_PERIOD_LOG) n = I2C_BB_SCL_PERIOD_LOG;
    LOG_INFO("I2CSlave: start=%lu stop=%lu match=%lu miss=%lu wr=%lu rd=%lu ack=%lu "
             "bus_to=%lu "
             "p(%u): %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu",
             (uint32_t)i2c_bb_slave_stats.starts,
             (uint32_t)i2c_bb_slave_stats.stops,
             (uint32_t)i2c_bb_slave_stats.addr_matches,
             (uint32_t)i2c_bb_slave_stats.addr_misses,
             (uint32_t)i2c_bb_slave_stats.writes,
             (uint32_t)i2c_bb_slave_stats.reads,
             (uint32_t)i2c_bb_slave_stats.acks_sent,
             (uint32_t)i2c_bb_slave_stats.scl_timeouts,
             (unsigned)n,
             (uint32_t)p[0],  (uint32_t)p[1],  (uint32_t)p[2],  (uint32_t)p[3],
             (uint32_t)p[4],  (uint32_t)p[5],  (uint32_t)p[6],  (uint32_t)p[7],
             (uint32_t)p[8],  (uint32_t)p[9],  (uint32_t)p[10], (uint32_t)p[11],
             (uint32_t)p[12], (uint32_t)p[13], (uint32_t)p[14], (uint32_t)p[15]);

    return 5000;


}

/* -------------------------------------------------------------------------
 * Write callback (called from ISR — keep short!)
 * ---------------------------------------------------------------------- */

void I2CSlaveThread::onWrite(uint8_t addr, const uint8_t *buf, uint8_t len)
{
    if (!s_instance || len == 0)
        return;


    if (addr == 0x50 || addr == 0x51) {
        uint8_t idx   = (addr == 0x51) ? 1 : 0;
        uint8_t start = buf[0];
        s_eeprom_ptr[idx] = start;
        for (uint8_t i = 1; i < len; i++) {
            s_instance->eeprom[idx][s_eeprom_ptr[idx]++] = buf[i];
        }
        if (len > 1) {
            s_write_addr    = addr;
            s_write_start   = start;
            s_write_count   = len - 1;
            s_write_pending = true;
            s_instance->setIntervalFromNow(0);
        }
    } else if (addr == 0x40) {
        if (buf[0] < REG_COUNT)
            s_reg_ptr = buf[0];
    }
}

/* -------------------------------------------------------------------------
 * Read callback (called from ISR — keep short!)
 * ---------------------------------------------------------------------- */

uint8_t I2CSlaveThread::onRead(uint8_t addr, uint8_t *buf, uint8_t max_len)
{
    if (!s_instance)
        return 0;

    if (addr == 0x50 || addr == 0x51) {
        uint8_t idx = (addr == 0x51) ? 1 : 0;
        uint8_t len = max_len < EEPROM_SIZE ? max_len : EEPROM_SIZE;
        for (uint8_t i = 0; i < len; i++) {
            buf[i] = s_instance->eeprom[idx][s_eeprom_ptr[idx]++];
        }
        return len;
    } else if (addr == 0x40) {
        if (s_reg_ptr >= REG_COUNT)
            return 0;
        uint8_t sz = s_instance->reg_size[s_reg_ptr];
        if (sz > REG_MAX)   sz = REG_MAX;   /* cap at 4 bytes */
        if (sz > max_len)   sz = max_len;
        memcpy(buf, s_instance->reg_data[s_reg_ptr], sz);
        return sz;
    }

    return 0;
}

#endif /* ALMEMO_SENSOR_RECEIVER */
