#ifndef I2C_TWIS_SLAVE_H
#define I2C_TWIS_SLAVE_H

#include <stdint.h>
#include <stdbool.h>
#include "nrf_gpio.h"

/*
 * Hardware I2C slave for nRF52840 using the TWIS0 and TWIS1 peripherals,
 * both routed to the same SDA/SCL pins.  Three 7-bit addresses are supported:
 *
 *   TWIS0 : ADDRESS[0] = 0x50
 *           ADDRESS[1] = 0x51
 *   TWIS1 : ADDRESS[0] = 0x40
 *
 * Both peripherals are listeners on the same bus.  Only the one that matches
 * the master's address drives SDA (open-drain) for ACK / data, so there is no
 * electrical contention.
 *
 * IRQ handlers SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler and
 * SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler are installed as strong
 * symbols; this overrides the weak aliases from startup.S via
 * -Wl,--allow-multiple-definition.  The names look SPI-related but are the
 * *hardware-instance* IRQ lines — Meshtastic's radio SPI uses SPIM3 on its
 * own dedicated IRQ line (SPIM3_IRQn) and is unaffected.
 */

#ifndef I2C_TWIS_SDA_PIN
#define I2C_TWIS_SDA_PIN      NRF_GPIO_PIN_MAP(1, 11)
#endif

#ifndef I2C_TWIS_SCL_PIN
#define I2C_TWIS_SCL_PIN      NRF_GPIO_PIN_MAP(1, 12)
#endif

#ifndef I2C_TWIS_IRQ_PRIORITY
#define I2C_TWIS_IRQ_PRIORITY 2
#endif

/* Buffer sizes large enough for a full 24Cxx-style EEPROM write
 * (1 address byte + 256 data bytes). */
#ifndef I2C_TWIS_RX_BUF_SIZE
#define I2C_TWIS_RX_BUF_SIZE  260u
#endif

#ifndef I2C_TWIS_TX_BUF_SIZE
#define I2C_TWIS_TX_BUF_SIZE  260u
#endif

typedef void    (*i2c_twis_write_cb_t)(uint8_t addr, const uint8_t *data, uint8_t len);
typedef uint8_t (*i2c_twis_read_cb_t) (uint8_t addr, uint8_t *buf,  uint8_t max_len);

typedef struct {
    volatile uint32_t writes;         /**< Completed master-write transactions. */
    volatile uint32_t reads;          /**< READ events dispatched.              */
    volatile uint32_t stopped;        /**< STOPPED events seen.                 */
    volatile uint32_t errors;         /**< ERROR events (ERRORSRC logged).      */
    volatile uint32_t last_errorsrc;  /**< Last non-zero ERRORSRC bitmask.      */
    volatile uint32_t match_0x50;
    volatile uint32_t match_0x51;
    volatile uint32_t match_0x40;
} i2c_twis_slave_stats_t;

extern volatile i2c_twis_slave_stats_t i2c_twis_slave_stats;

/** Initialise TWIS0 + TWIS1, enable both IRQs at priority 2. */
void i2c_twis_slave_init(void);

/**
 * Register callbacks for a 7-bit slave address.  Must be one of 0x50, 0x51, 0x40.
 * @return true on success, false if the address is not one of the configured slots.
 */
bool i2c_twis_slave_register(uint8_t addr,
                              i2c_twis_write_cb_t on_write,
                              i2c_twis_read_cb_t  on_read);

#endif /* I2C_TWIS_SLAVE_H */
