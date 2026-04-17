#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

/**
 * @file  i2c_slave.h
 * @brief Bitbang I2C Slave library for nRF52840
 *
 * Features:
 *  - GPIO bitbanging, edge-triggered via GPIOTE PORT events (GPIO SENSE)
 *  - Multiple slave addresses
 *  - Clock stretching support
 *  - Callbacks: on_write / on_read
 *
 * Configuration (override via compiler flags or before including this header):
 *
 *   I2C_SDA        nRF52840 raw pin number for SDA (e.g. 43 for P1.11)
 *   I2C_SCL        nRF52840 raw pin number for SCL (e.g. 44 for P1.12)
 *   I2C_MAX_ADDRS  Max number of slave addresses (default: 4)
 *   I2C_BUF_SIZE   Internal RX/TX buffer size    (default: 64)
 *
 * Usage:
 *   1. Call i2c_slave_init() and i2c_slave_add_address().
 *   2. From C++, install a GPIOTE_IRQHandler that calls i2c_slave_irq_handler()
 *      when EVENTS_PORT is set (see I2CSlaveThread.cpp for the wrapper).
 *   3. Call i2c_slave_start() to arm GPIO SENSE and enable the PORT interrupt.
 */

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------
 * Configuration defaults
 * ---------------------------------------------------------------------- */
#ifndef I2C_SDA
#  define I2C_SDA   999 //default will never work to break compiling
#endif

#ifndef I2C_SCL
#  define I2C_SCL   999 //default will never work to break compiling
#endif

#ifndef I2C_MAX_ADDRS
#  define I2C_MAX_ADDRS  4
#endif

#ifndef I2C_BUF_SIZE
#  define I2C_BUF_SIZE   64
#endif

/* -------------------------------------------------------------------------
 * Callback types
 * ---------------------------------------------------------------------- */

/**
 * @brief Called after a complete I2C write transaction.
 */
typedef void (*i2c_slave_on_write_t)(uint8_t addr,
                                     const uint8_t *buf,
                                     uint8_t len);

/**
 * @brief Called when the master requests a read from this slave.
 *        Fill @p buf with up to @p max_len bytes.
 * @return Number of bytes placed in buf (0 … max_len)
 */
typedef uint8_t (*i2c_slave_on_read_t)(uint8_t addr,
                                       uint8_t *buf,
                                       uint8_t max_len);

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

/**
 * @brief Initialise the I2C slave library.
 *
 * Configures SDA/SCL as input with pull-up (no SENSE yet) and registers
 * callbacks.  Call i2c_slave_start() afterwards to arm the PORT interrupt.
 */
void i2c_slave_init(i2c_slave_on_write_t on_write,
                    i2c_slave_on_read_t  on_read);

/**
 * @brief Arm GPIO SENSE on SDA/SCL and enable the GPIOTE PORT interrupt.
 *
 * Must be called after installing the GPIOTE wrapper (NVIC_SetVector) and
 * before any I2C traffic is expected.
 */
void i2c_slave_start(void);

/**
 * @brief PORT event handler — call from GPIOTE_IRQHandler when EVENTS_PORT is set.
 *
 * Reads the GPIO LATCH registers to determine which pin triggered, updates
 * SENSE polarity for the next edge, and dispatches to the state machine.
 */
void i2c_slave_irq_handler(void);

/**
 * @brief Register a 7-bit slave address to respond to.
 * @return true on success, false if the address table is full
 */
bool i2c_slave_add_address(uint8_t addr);

/**
 * @brief Remove a previously registered slave address.
 */
void i2c_slave_remove_address(uint8_t addr);

/**
 * @brief Disable SENSE on both pins and reset the state machine.
 */
void i2c_slave_disable(void);

/**
 * @brief HW test: pull SDA low (1) or release (0).
 */
void i2c_slave_test_sda(uint8_t pull_low);

/**
 * @brief Debug counters — incremented from ISR, read from main loop.
 */
typedef struct {
    volatile uint32_t scl_edges;
    volatile uint32_t sda_edges;
    volatile uint32_t starts;
    volatile uint32_t stops;
    volatile uint32_t addr_matches;
    volatile uint32_t addr_misses;
    volatile uint32_t writes;
    volatile uint32_t reads;
    volatile uint32_t acks_sent;
    volatile uint32_t addr_duration;  /**< DWT ticks: SCL bit1↑ to bit8↑ */
    volatile uint32_t sda_low_ticks;  /**< DWT ticks SDA held LOW for ACK */

#define I2C_SCL_PERIOD_LOG 16
    volatile uint32_t scl_periods[I2C_SCL_PERIOD_LOG];
    volatile uint8_t  scl_period_idx;
} i2c_slave_stats_t;

extern volatile i2c_slave_stats_t i2c_slave_stats;

#endif /* I2C_SLAVE_H */
