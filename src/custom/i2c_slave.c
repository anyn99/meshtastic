/**
 * @file  i2c_slave.c
 * @brief Bitbang I2C Slave implementation for nRF52840
 *
 * Uses nrf_gpio.h HAL for open-drain GPIO (P0 and P1 transparently handled).
 * Interrupts are driven by GPIOTE PORT events (GPIO SENSE mechanism) — no
 * dedicated GPIOTE channels are used, so SDA and SCL can be freely switched
 * between input (SENSE active) and output (SENSE disabled) at any time.
 *
 * Call i2c_slave_init() once, then i2c_slave_start() to arm the pins and
 * enable the GPIOTE PORT interrupt.  From C++ install a GPIOTE_IRQHandler
 * that calls i2c_slave_irq_handler() for PORT events.
 *
 * Pin drive model
 * ---------------
 * Both SDA and SCL are open-drain: configured as INPUT with pull-up (idle /
 * released) or OUTPUT LOW (asserted). Never drive high.
 *
 * Clock stretching / ACK
 * ----------------------
 * Before asserting SDA or SCL low the SENSE field is cleared (NOSENSE) so
 * reconfiguring the pin as an output does not produce spurious PORT events.
 * pin_release restores INPUT+PULLUP and re-arms SENSE for the next edge.
 */

#include "i2c_slave.h"

#include <nrf_gpio.h>
#include <core_cm4.h>   /* DWT->CYCCNT — Cortex-M4 cycle counter */

#include <string.h>
#include <stddef.h>

/* -------------------------------------------------------------------------
 * Port / bit helpers (nRF52840: pins 0-31 → P0, pins 32-47 → P1)
 * ---------------------------------------------------------------------- */

#define PIN_PORT(p)  ((p) < 32u ? NRF_P0 : NRF_P1)
#define PIN_BIT(p)   (1u << ((p) & 0x1Fu))

/* -------------------------------------------------------------------------
 * Sense state tracking — records which edge each pin is currently waiting for.
 * SENSE_HIGH = waiting for rising edge, SENSE_LOW = waiting for falling edge.
 * ---------------------------------------------------------------------- */

static volatile nrf_gpio_pin_sense_t s_scl_sense;
static volatile nrf_gpio_pin_sense_t s_sda_sense;

/* -------------------------------------------------------------------------
 * GPIO helpers (open-drain emulation via nrf_gpio HAL)
 * ---------------------------------------------------------------------- */

static inline uint8_t pin_read(uint8_t pin)
{
    return (uint8_t)nrf_gpio_pin_read(pin);
}

/* Disable SENSE, then drive output low (open-drain: S0D1). */
static inline void pin_pull_low(uint8_t pin)
{
    nrf_gpio_cfg_sense_set(pin, NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg(pin,
                 NRF_GPIO_PIN_DIR_OUTPUT,
                 NRF_GPIO_PIN_INPUT_DISCONNECT,
                 NRF_GPIO_PIN_NOPULL,
                 GPIO_PIN_CNF_DRIVE_S0D1,
                 NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_pin_clear(pin);
}

/* Release SCL: INPUT+PULLUP, re-arm SENSE for next edge, update s_scl_sense. */
static inline void scl_release(void)
{
    nrf_gpio_cfg_input(I2C_SCL, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_pin_sense_t sense = pin_read(I2C_SCL)
                                 ? NRF_GPIO_PIN_SENSE_LOW
                                 : NRF_GPIO_PIN_SENSE_HIGH;
    s_scl_sense = sense;
    nrf_gpio_cfg_sense_set(I2C_SCL, sense);
}

/* Release SDA: INPUT+PULLUP, re-arm SENSE for next edge, update s_sda_sense. */
static inline void sda_release(void)
{
    nrf_gpio_cfg_input(I2C_SDA, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_pin_sense_t sense = pin_read(I2C_SDA)
                                 ? NRF_GPIO_PIN_SENSE_LOW
                                 : NRF_GPIO_PIN_SENSE_HIGH;
    s_sda_sense = sense;
    nrf_gpio_cfg_sense_set(I2C_SDA, sense);
}

static inline void scl_stretch(void)  { pin_pull_low(I2C_SCL); }
static inline void sda_pull_low(void) { pin_pull_low(I2C_SDA); }

/* -------------------------------------------------------------------------
 * State machine
 * ---------------------------------------------------------------------- */

typedef enum {
    STATE_IDLE,
    STATE_ADDR,        /* Receiving address + R/W bit */
    STATE_ACK_ADDR,    /* Sending ACK after address */
    STATE_DATA_WRITE,  /* Receiving data bytes (master → slave) */
    STATE_ACK_DATA_W,  /* Sending ACK after each write byte */
    STATE_DATA_READ,   /* Sending data bytes (slave → master) */
    STATE_ACK_DATA_R,  /* Receiving ACK/NACK from master after read byte */
} i2c_state_t;

/* -------------------------------------------------------------------------
 * Module state
 * ---------------------------------------------------------------------- */

volatile i2c_slave_stats_t i2c_slave_stats;

static i2c_slave_on_write_t s_on_write;
static i2c_slave_on_read_t  s_on_read;

static uint8_t  s_addresses[I2C_MAX_ADDRS];
static uint8_t  s_addr_count;

static volatile i2c_state_t s_state;

static uint8_t  s_shift_reg;
static uint8_t  s_bit_count;

static uint8_t  s_matched_addr;
static bool     s_is_read;

static uint8_t  s_rx_buf[I2C_BUF_SIZE];
static uint8_t  s_rx_len;

static uint8_t  s_tx_buf[I2C_BUF_SIZE];
static uint8_t  s_tx_len;
static uint8_t  s_tx_pos;

/* DWT timestamp of the 1st SCL rising edge of the address byte */
static uint32_t s_addr_t0;
/* DWT timestamp when SDA was pulled LOW for ACK (pre-pull) */
static uint32_t s_sda_low_t0;
/* DWT timestamp of the previous SCL rising edge (for period measurement) */
static uint32_t s_scl_t_prev_rise;

/* -------------------------------------------------------------------------
 * Helpers
 * ---------------------------------------------------------------------- */

static bool addr_is_registered(uint8_t addr)
{
    for (uint8_t i = 0; i < s_addr_count; i++) {
        if (s_addresses[i] == addr) return true;
    }
    return false;
}

static void reset_to_idle(void)
{
    s_state     = STATE_IDLE;
    s_bit_count = 0;
    s_shift_reg = 0;
    s_rx_len    = 0;
    s_tx_pos    = 0;
    s_tx_len    = 0;
    sda_release();
    scl_release();
}

static void handle_start(void)
{
    i2c_slave_stats.starts++;
    s_state     = STATE_ADDR;
    s_bit_count = 0;
    s_shift_reg = 0;
    s_rx_len    = 0;
    s_tx_pos    = 0;
    s_tx_len    = 0;
    /* Reset SCL period log for this transaction */
    i2c_slave_stats.scl_period_idx = 0;
    s_scl_t_prev_rise = 0;
}

static void handle_stop(void)
{
    if (s_state == STATE_DATA_WRITE || s_state == STATE_ACK_DATA_W) {
        i2c_slave_stats.writes++;
        if (s_on_write && s_rx_len > 0)
            s_on_write(s_matched_addr, s_rx_buf, s_rx_len);
    } else if (s_state == STATE_DATA_READ || s_state == STATE_ACK_DATA_R) {
        i2c_slave_stats.reads++;
    }
    i2c_slave_stats.stops++;
    reset_to_idle();
}

/* Called on SCL rising edge — NEVER modify SDA here (SCL is HIGH). */
static void on_scl_rising(uint8_t sda)
{
    /* Record period between consecutive rising edges */
    uint32_t now = DWT->CYCCNT;
    if (s_scl_t_prev_rise) {
        uint8_t idx = i2c_slave_stats.scl_period_idx;
        if (idx < I2C_SCL_PERIOD_LOG)
            i2c_slave_stats.scl_periods[idx] = now - s_scl_t_prev_rise;
        i2c_slave_stats.scl_period_idx = idx + 1;
    }
    s_scl_t_prev_rise = now;

    switch (s_state) {

    case STATE_ADDR:
        s_shift_reg = (s_shift_reg << 1) | sda;
        if (s_bit_count == 0)
            s_addr_t0 = DWT->CYCCNT;
        if (++s_bit_count == 8) {
            uint8_t addr = s_shift_reg >> 1;
            s_is_read    = s_shift_reg & 0x01;
            s_bit_count  = 0;
            s_shift_reg  = 0;
            i2c_slave_stats.addr_duration = DWT->CYCCNT - s_addr_t0;
            if (addr_is_registered(addr)) {
                i2c_slave_stats.addr_matches++;
                s_matched_addr = addr;
                s_state        = STATE_ACK_ADDR;
                if (!s_is_read) {
                    s_sda_low_t0 = DWT->CYCCNT;
                    sda_pull_low();
                }
            } else {
                i2c_slave_stats.addr_misses++;
                s_state = STATE_IDLE;
            }
        }
        break;

    case STATE_ACK_ADDR:
        /* 9th SCL high: master reads our ACK. No-op. */
        break;

    case STATE_DATA_WRITE:
        s_shift_reg = (s_shift_reg << 1) | sda;
        if (++s_bit_count == 8) {
            s_bit_count = 0;
            if (s_rx_len < I2C_BUF_SIZE)
                s_rx_buf[s_rx_len++] = s_shift_reg;
            s_shift_reg = 0;
            s_state     = STATE_ACK_DATA_W;
        }
        break;

    case STATE_ACK_DATA_W:
        /* 9th SCL high: master reads our ACK. No-op. */
        break;

    case STATE_DATA_READ:
        if (++s_bit_count == 8) {
            s_bit_count = 0;
            s_tx_pos++;
            s_state = STATE_ACK_DATA_R;
        }
        break;

    case STATE_ACK_DATA_R:
        if (sda == 0)
            s_state = STATE_DATA_READ;
        else
            s_state = STATE_IDLE;
        s_bit_count = 0;
        break;

    default:
        break;
    }
}

/* Called on SCL falling edge — safe to modify SDA here (SCL is LOW). */
static void on_scl_falling(void)
{
    switch (s_state) {

    case STATE_ACK_ADDR:
        if (s_bit_count == 0) {
            if (s_is_read) {
                scl_stretch();
                s_tx_len = s_on_read ? s_on_read(s_matched_addr, s_tx_buf, I2C_BUF_SIZE) : 0;
                s_tx_pos = 0;
                sda_pull_low();
                scl_release();
            } else {
                scl_stretch();
                sda_pull_low();
                scl_release();
                i2c_slave_stats.acks_sent++;
            }
            s_bit_count = 1;
        } else {
            if (s_is_read) {
                s_state     = STATE_DATA_READ;
                s_bit_count = 0;
                if (s_tx_len > 0)
                    (s_tx_buf[0] & 0x80) ? sda_release() : sda_pull_low();
                else
                    sda_release();
            } else {
                i2c_slave_stats.sda_low_ticks = DWT->CYCCNT - s_sda_low_t0;
                sda_release();
                s_state     = STATE_DATA_WRITE;
                s_bit_count = 0;
            }
        }
        break;

    case STATE_ACK_DATA_W:
        if (s_bit_count == 0) {
            scl_stretch();
            sda_pull_low();
            scl_release();
            s_bit_count = 1;
        } else {
            sda_release();
            s_state     = STATE_DATA_WRITE;
            s_bit_count = 0;
        }
        break;

    case STATE_ACK_DATA_R:
        sda_release();
        break;

    case STATE_DATA_READ:
        if (s_tx_pos < s_tx_len) {
            uint8_t bit_idx = 7 - s_bit_count;
            ((s_tx_buf[s_tx_pos] >> bit_idx) & 1) ? sda_release() : sda_pull_low();
        } else {
            sda_release();
        }
        break;

    default:
        break;
    }
}

/* -------------------------------------------------------------------------
 * PORT event ISR — call from GPIOTE_IRQHandler when EVENTS_PORT is set
 * ---------------------------------------------------------------------- */

void i2c_slave_irq_handler(void)
{
    /* Clear PORT event immediately to re-arm future events */
    NRF_GPIOTE->EVENTS_PORT = 0;
    __DSB();

    NRF_GPIO_Type *scl_port = PIN_PORT(I2C_SCL);
    NRF_GPIO_Type *sda_port = PIN_PORT(I2C_SDA);
    uint32_t       scl_bit  = PIN_BIT(I2C_SCL);
    uint32_t       sda_bit  = PIN_BIT(I2C_SDA);

    /* Read LATCH to find which pin(s) triggered.
     * LATCH bit is set when a SENSE pin's DETECT signal goes 0→1, i.e. the
     * pin matched its configured polarity. Clear by writing 1 to the bit. */
    uint32_t latch_scl = scl_port->LATCH & scl_bit;
    uint32_t latch_sda = sda_port->LATCH & sda_bit;

    if (latch_scl) scl_port->LATCH = scl_bit;
    if (latch_sda) sda_port->LATCH = sda_bit;

    /* --- SCL edge -------------------------------------------------------- */
    if (latch_scl) {
        i2c_slave_stats.scl_edges++;
        /* s_scl_sense holds the polarity we were waiting for → that edge fired. */
        bool rising = (s_scl_sense == NRF_GPIO_PIN_SENSE_HIGH);
        /* Flip SENSE to catch the opposite edge next. */
        s_scl_sense = rising ? NRF_GPIO_PIN_SENSE_LOW : NRF_GPIO_PIN_SENSE_HIGH;
        nrf_gpio_cfg_sense_set(I2C_SCL, s_scl_sense);

        if (rising)
            on_scl_rising(pin_read(I2C_SDA));
        else
            on_scl_falling();
        /* Note: on_scl_falling may call scl_stretch()/scl_release() which
         * disable/re-enable SCL SENSE and update s_scl_sense accordingly. */
    }

    /* --- SDA edge (START / STOP detection) -------------------------------- */
    if (latch_sda) {
        i2c_slave_stats.sda_edges++;
        bool rising = (s_sda_sense == NRF_GPIO_PIN_SENSE_HIGH);
        s_sda_sense = rising ? NRF_GPIO_PIN_SENSE_LOW : NRF_GPIO_PIN_SENSE_HIGH;

        /* Only re-arm SENSE if SDA is still an INPUT.
         * If the SCL handler above called sda_pull_low(), SDA is now an OUTPUT
         * (DIR=1).  Setting SENSE on an OUTPUT LOW pin with SENSE_LOW would
         * make DETECT=1 immediately → ISR storm.  sda_release() will re-arm
         * SENSE with the correct polarity once SDA returns to INPUT. */
        if (!(sda_port->DIR & sda_bit))
            nrf_gpio_cfg_sense_set(I2C_SDA, s_sda_sense);

        /* START / STOP conditions: SDA changes while SCL is HIGH */
        if (pin_read(I2C_SCL)) {
            if (!rising) {           /* SDA fell while SCL high → START */
                if (s_state == STATE_IDLE)
                    handle_start();
            } else {                 /* SDA rose while SCL high → STOP */
                handle_stop();
            }
        }
    }
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

void i2c_slave_init(i2c_slave_on_write_t on_write,
                    i2c_slave_on_read_t  on_read)
{
    s_on_write   = on_write;
    s_on_read    = on_read;
    s_addr_count = 0;

    memset(s_addresses, 0, sizeof(s_addresses));
    memset(s_rx_buf,    0, sizeof(s_rx_buf));
    memset(s_tx_buf,    0, sizeof(s_tx_buf));

    /* Basic pin config (no SENSE yet — i2c_slave_start() arms SENSE). */
    nrf_gpio_cfg_input(I2C_SDA, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(I2C_SCL, NRF_GPIO_PIN_PULLUP);

    /* Enable DWT cycle counter for ISR timing measurements */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;

    s_state = STATE_IDLE;
}

void i2c_slave_start(void)
{
    /* Arm SENSE on both pins based on their current level.
     * Bus idle → both HIGH → detect next falling edge (SENSE_LOW).
     * If bus is mid-transaction this still sets a sane starting state. */
    s_sda_sense = pin_read(I2C_SDA) ? NRF_GPIO_PIN_SENSE_LOW : NRF_GPIO_PIN_SENSE_HIGH;
    nrf_gpio_cfg_sense_input(I2C_SDA, NRF_GPIO_PIN_PULLUP, s_sda_sense);

    s_scl_sense = pin_read(I2C_SCL) ? NRF_GPIO_PIN_SENSE_LOW : NRF_GPIO_PIN_SENSE_HIGH;
    nrf_gpio_cfg_sense_input(I2C_SCL, NRF_GPIO_PIN_PULLUP, s_scl_sense);

    /* Clear any stale LATCH bits that may have been set before SENSE was armed,
     * then clear the PORT event and enable the interrupt. */
    PIN_PORT(I2C_SDA)->LATCH = PIN_BIT(I2C_SDA);
    PIN_PORT(I2C_SCL)->LATCH = PIN_BIT(I2C_SCL);
    NRF_GPIOTE->EVENTS_PORT = 0;
    NRF_GPIOTE->INTENSET    = GPIOTE_INTENSET_PORT_Msk;
    NVIC_EnableIRQ(GPIOTE_IRQn);
}

bool i2c_slave_add_address(uint8_t addr)
{
    if (s_addr_count >= I2C_MAX_ADDRS) return false;
    s_addresses[s_addr_count++] = addr & 0x7F;
    return true;
}

void i2c_slave_remove_address(uint8_t addr)
{
    addr &= 0x7F;
    for (uint8_t i = 0; i < s_addr_count; i++) {
        if (s_addresses[i] == addr) {
            for (uint8_t j = i; j < s_addr_count - 1; j++)
                s_addresses[j] = s_addresses[j + 1];
            s_addr_count--;
            return;
        }
    }
}

void i2c_slave_disable(void)
{
    /* Disable SENSE on both pins so PORT events stop firing for us. */
    nrf_gpio_cfg_sense_set(I2C_SDA, NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg_sense_set(I2C_SCL, NRF_GPIO_PIN_NOSENSE);
    reset_to_idle();
}

void i2c_slave_test_sda(uint8_t pull_low)
{
    if (pull_low)
        sda_pull_low();
    else
        sda_release();
}
