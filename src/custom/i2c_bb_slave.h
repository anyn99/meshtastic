#ifndef I2C_BB_SLAVE_H
#define I2C_BB_SLAVE_H

#include <stdint.h>
#include <stdbool.h>
#include "nrf_gpio.h"   /* Required for NRF_GPIO_PIN_MAP */

// ─── Configuration ────────────────────────────────────────────────────────────

#ifndef I2C_BB_SDA_PIN
#define I2C_BB_SDA_PIN      NRF_GPIO_PIN_MAP(1, 11)
#endif

#ifndef I2C_BB_SCL_PIN
#define I2C_BB_SCL_PIN      NRF_GPIO_PIN_MAP(1, 12)
#endif

/** Maximum number of slave addresses that can be registered. */
#ifndef I2C_BB_MAX_ADDRESSES
#define I2C_BB_MAX_ADDRESSES  4
#endif

/**
 * GPIOTE PORT event IRQ priority.
 * Must be high enough to bit-bang in time, but leave room for SoftDevice if used.
 */
#ifndef I2C_BB_IRQ_PRIORITY
#define I2C_BB_IRQ_PRIORITY   2
#endif

// ─── Types ────────────────────────────────────────────────────────────────────

/**
 * Called when the master completes a WRITE transaction to one of our addresses.
 * Also called before a Repeated START if a write was in progress.
 *
 * @param addr  The matched 7-bit slave address.
 * @param data  Received byte buffer (excluding address byte).
 * @param len   Number of bytes received.
 */
typedef void (*i2c_bb_write_cb_t)(uint8_t addr, const uint8_t *data, uint8_t len);

/**
 * Called when the master performs a READ from one of our addresses.
 * Fill @p buf with up to @p max_len bytes; return actual byte count.
 *
 * @param addr      The matched 7-bit slave address.
 * @param buf       Buffer to fill with response data.
 * @param max_len   Maximum bytes the master will clock out.
 * @return          Number of bytes placed in @p buf (clamped to max_len).
 */
typedef uint8_t (*i2c_bb_read_cb_t)(uint8_t addr, uint8_t *buf, uint8_t max_len);

/** Entry for one registered slave address. */
typedef struct {
    uint8_t           addr;     /**< 7-bit I2C address (0x00 = General Call, valid). */
    bool              active;   /**< true = slot is in use.                           */
    i2c_bb_write_cb_t on_write;
    i2c_bb_read_cb_t  on_read;
} i2c_bb_addr_entry_t;

// ─── Debug statistics ─────────────────────────────────────────────────────────

/** Number of SCL periods kept in the ring buffer. */
#define I2C_BB_SCL_PERIOD_LOG 16

/** Number of recent IRQ durations kept for the moving-window average. */
#define I2C_BB_IRQ_DUR_LOG 10

/**
 * Per-pin bus-stuck timeout in microseconds.
 * If SCL has not changed for this long, or SDA has not changed for this long,
 * while the state machine is not IDLE, the bus is assumed stuck and is
 * forcibly reset to IDLE.
 */
#ifndef I2C_BB_TIMEOUT_US
#define I2C_BB_TIMEOUT_US  500u
#endif

/**
 * Timeout converted to DWT cycles.
 * nRF52840 runs at 64 MHz → 15.625 ns per cycle → 64 cycles per microsecond.
 * The clock frequency is fixed for this family so the factor is a constant.
 */
#define I2C_BB_TIMEOUT_CYCLES  ((I2C_BB_TIMEOUT_US) * 64u)

typedef struct {
    volatile uint32_t starts;       /**< START / Repeated START conditions seen. */
    volatile uint32_t stops;        /**< STOP conditions seen.                   */
    volatile uint32_t addr_matches; /**< Address matched — ACK driven.           */
    volatile uint32_t addr_misses;  /**< Address not matched — NACK / idle.      */
    volatile uint32_t writes;       /**< Write transactions committed.           */
    volatile uint32_t reads;        /**< Read transactions served.               */
    volatile uint32_t acks_sent;    /**< Data-byte ACKs driven to master.        */
    volatile uint32_t scl_timeouts; /**< Resets triggered by stuck-bus timeout.  */
    volatile uint32_t mid_tx_aborts;   /**< Timeouts that fired while a read or write transaction was in progress (tx_idx>0 or rx_len>0). */
    volatile uint32_t spurious_stops;  /**< STOP detected while state machine was still in a READ data/ack phase (master should have ended cleanly via NACK first). */
    volatile uint32_t anomaly119_irqs; /**< IRQs entered with no LATCH bit set anywhere → true spurious / nRF52 Anomaly 119 fire. */
    volatile uint32_t irq_entries;     /**< Total entries into the IRQ handler (EVENTS_PORT != 0). Lets us see whether IRQs are still firing while a state appears stuck. */
    volatile uint32_t write_edge_loss; /**< Defensive ST_WRITE_POST_ACK falling-edge recovery (missed ACK rising). */
    volatile uint32_t read_edge_loss;  /**< Defensive ST_READ_LAST_BIT falling-edge recovery (missed last-bit rising). */
    volatile uint32_t irq_dur_cyc[I2C_BB_IRQ_DUR_LOG]; /**< Ring of recent IRQ durations in DWT cycles (64 MHz → /64 = µs); reader derives min/avg/max from this. */
    volatile uint8_t  irq_dur_idx;                      /**< Next write index (wraps at I2C_BB_IRQ_DUR_LOG). */
    volatile uint32_t scl_periods[I2C_BB_SCL_PERIOD_LOG]; /**< DWT ticks between SCL rising edges. */
    volatile uint8_t  scl_period_idx;                      /**< Next write index (wraps at I2C_BB_SCL_PERIOD_LOG). */
} i2c_bb_slave_stats_t;

extern volatile i2c_bb_slave_stats_t i2c_bb_slave_stats;

// ─── API ──────────────────────────────────────────────────────────────────────

/** Initialise GPIO and GPIOTE PORT events. Call once at startup. */
void i2c_bb_slave_init(void);

/**
 * Register a 7-bit slave address with its callbacks.
 * Registering the same address again updates the callbacks.
 *
 * @return true on success, false if the address table is full.
 */
bool i2c_bb_slave_register(uint8_t           addr,
                            i2c_bb_write_cb_t on_write,
                            i2c_bb_read_cb_t  on_read);

/** Unregister a previously registered slave address. */
void i2c_bb_slave_unregister(uint8_t addr);

/** Returns true when the state machine is in its idle (between transactions) state. */
bool i2c_bb_slave_is_idle(void);

/** Returns a static string naming the current state-machine state ("ST_IDLE", "ST_READ_TX", ...). */
const char *i2c_bb_slave_state_name(void);

/** Snapshot of state-machine internals for stuck-state debugging.
 *  @param bit_cnt  Remaining bits to clock in the current byte (8 = none clocked yet).
 *  @param rx_len   Number of received data bytes in the pending write buffer.
 *  @param tx_idx   Number of bytes already transmitted in the current read.
 */
void i2c_bb_slave_get_debug(uint8_t *bit_cnt, uint8_t *rx_len, uint8_t *tx_idx);

/**
 * Call from your GPIOTE_IRQHandler if you manage the IRQ yourself.
 * If I2C_BB_OWN_IRQ is NOT defined the driver installs its own handler
 * and this function need not be called manually.
 */
void i2c_bb_slave_gpiote_irq_handler(void);

#endif /* I2C_BB_SLAVE_H */
