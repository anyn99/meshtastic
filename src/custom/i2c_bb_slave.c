#if defined(ALMEMO_SENSOR_RECEIVER)

/**
 * @file  i2c_bb_slave.c
 * @brief Bit-bang I2C slave for nRF52840.
 *
 * Edge semantics (I2C spec):
 *   Rising  SCL  →  sample SDA          (data valid)
 *   Falling SCL  →  change SDA          (setup next bit / drive ACK)
 *
 * Clock stretching on the FALLING edge:
 *   scl_low() immediately → process → stretch_end() → scl_release()
 *
 * START / STOP detection strategy:
 *   SDA SENSE is enabled ONLY while SCL is high (the only window where
 *   START/STOP can legally occur).  On every SCL falling edge stretch_begin()
 *   disables SDA SENSE before we touch SDA, preventing any data/ACK transitions
 *   from being misread as START/STOP conditions.
 *
 *   SDA SENSE direction is set to match the current SDA level when we arm it,
 *   so it fires on the very next SDA transition regardless of direction.
 *
 * Write data is committed to the callback:
 *   - at STOP
 *   - at Repeated START (before re-entering address phase)
 *
 * No GPIOTE IN channels are used → SDA direction is always freely switchable.
 */

#include "i2c_bb_slave.h"

#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"

#include <string.h>

// ─── Internal constants ────────────────────────────────────────────────────────

#define I2C_BB_MAX_BUF  64u

#define _PIN_PORT(p)  ((p) < 32u ? NRF_P0 : NRF_P1)
#define _PIN_BIT(p)   ((p) < 32u ? (p) : (p) - 32u)

// ─── State machine ─────────────────────────────────────────────────────────────
//
//  Falling SCL (change SDA):   ST_ADDR_ACK, ST_WRITE_RX, ST_WRITE_ACK,
//                               ST_READ_ACK_WAIT, ST_READ_TX
//  Rising  SCL (sample SDA):   ST_ADDR_RX,  ST_WRITE_POST_ACK, ST_WRITE_RX,
//                               ST_READ_LAST_BIT, ST_READ_ACK_RX

typedef enum {
    ST_IDLE,
    ST_ADDR_RX,          /**< Rising:  sample address bits.                       */
    ST_ADDR_ACK,         /**< Falling: drive ACK/NACK for address byte.           */
    ST_WRITE_RX,         /**< Rising:  sample data bits from master.              */
    ST_WRITE_ACK,        /**< Falling: drive ACK (or NACK on overflow).           */
    ST_WRITE_POST_ACK,   /**< Rising:  ACK being sampled by master; do nothing.   */
    ST_READ_TX,          /**< Falling: output data bit (bits 7 down to 1).        */
    ST_READ_LAST_BIT,    /**< Rising:  master samples 8th bit; SDA released on next falling. */
    ST_READ_ACK_WAIT,    /**< Falling: release SDA, ACK low-phase; master drives ACK/NACK.  */
    ST_READ_ACK_RX,      /**< Rising:  sample master ACK/NACK.                    */
} i2c_bb_state_t;

// ─── Module state ──────────────────────────────────────────────────────────────

volatile i2c_bb_slave_stats_t i2c_bb_slave_stats;

static uint32_t s_last_scl_rise;         /**< DWT->CYCCNT at the last SCL rising edge.              */
static uint32_t s_last_state_change_cyc; /**< DWT->CYCCNT when SCL or SDA last latched (bus active). */

static bool s_scl_last; /**< Last observed SCL level (true = HIGH). Updated each IRQ. */
static bool s_sda_last; /**< Last observed SDA level (true = HIGH). Updated each IRQ. */

static i2c_bb_addr_entry_t s_addrs[I2C_BB_MAX_ADDRESSES];
static i2c_bb_state_t      s_state = ST_IDLE;

static uint8_t s_shift_reg;
static uint8_t s_bit_cnt;

static uint8_t s_rx_buf[I2C_BB_MAX_BUF];
static uint8_t s_rx_len;

static uint8_t s_tx_buf[I2C_BB_MAX_BUF];
static uint8_t s_tx_len;
static uint8_t s_tx_idx;

static uint8_t s_matched_addr;
static bool    s_is_read;

// ─── GPIO helpers ──────────────────────────────────────────────────────────────

static inline void sda_release(void)
{
    nrf_gpio_cfg_input(I2C_BB_SDA_PIN, NRF_GPIO_PIN_PULLUP);
}

static inline void sda_low(void)
{
    nrf_gpio_pin_clear(I2C_BB_SDA_PIN);
    nrf_gpio_cfg_output(I2C_BB_SDA_PIN);
}

static inline bool sda_read(void)
{
    return nrf_gpio_pin_read(I2C_BB_SDA_PIN) != 0;
}

static inline void scl_release(void)
{
    nrf_gpio_cfg_input(I2C_BB_SCL_PIN, NRF_GPIO_PIN_PULLUP);
}

static inline void scl_low(void)
{
    nrf_gpio_pin_clear(I2C_BB_SCL_PIN);
    nrf_gpio_cfg_output(I2C_BB_SCL_PIN);
}

static inline bool scl_read(void)
{
    return nrf_gpio_pin_read(I2C_BB_SCL_PIN) != 0;
}

// ─── SENSE helpers ─────────────────────────────────────────────────────────────

static void pin_sense_set(uint32_t pin, nrf_gpio_pin_sense_t sense)
{
    NRF_GPIO_Type *port = _PIN_PORT(pin);
    uint32_t       bit  = _PIN_BIT(pin);
    port->PIN_CNF[bit]  = (port->PIN_CNF[bit] & ~GPIO_PIN_CNF_SENSE_Msk)
                          | ((uint32_t)sense << GPIO_PIN_CNF_SENSE_Pos);
    port->LATCH         = (1u << bit);
}

static void pin_sense_disable(uint32_t pin)
{
    pin_sense_set(pin, NRF_GPIO_PIN_NOSENSE);
}

/**
 * Re-arm SDA SENSE for the next SDA transition.
 * @param current_high  Current (pre-read) SDA level — true = HIGH.
 * Arms for the OPPOSITE level so the event fires on the genuine next edge.
 * Called ONLY from the central re-arm block at the end of the IRQ handler.
 */
static void sda_sense_arm(bool current_high)
{
    pin_sense_set(I2C_BB_SDA_PIN,
        current_high ? NRF_GPIO_PIN_SENSE_LOW : NRF_GPIO_PIN_SENSE_HIGH);
}

/**
 * Re-arm SCL SENSE for the next SCL transition.
 * @param current_high  Current (pre-read) SCL level — true = HIGH.
 * Arms for the OPPOSITE level.  GPIOTE PORT events are LEVEL-sensitive;
 * arming for the same level as the current pin state causes an immediate
 * re-fire (nRF52 Anomaly 119).  Using the opposite level guarantees the
 * event fires only on a real edge.
 * Called ONLY from the central re-arm block at the end of the IRQ handler.
 */
static void scl_sense_arm(bool current_high)
{
    pin_sense_set(I2C_BB_SCL_PIN,
        current_high ? NRF_GPIO_PIN_SENSE_LOW : NRF_GPIO_PIN_SENSE_HIGH);
}

// ─── Clock stretching ──────────────────────────────────────────────────────────

static inline void stretch_begin(void)
{
    scl_low();
    /*
     * Disable SDA SENSE immediately on the falling edge, before we touch SDA.
     * This prevents any SDA transitions we make (ACK, data bits) from being
     * misinterpreted as START/STOP conditions.
     */
    pin_sense_disable(I2C_BB_SDA_PIN);
}

/**
 * Release SCL after clock-stretching.
 *
 * SCL SENSE is NOT set here.  The IRQ handler reads the current SCL/SDA
 * levels BEFORE calling the state machine; using that pre-read value for
 * re-arming at the end of the handler is race-free even after scl_release().
 */
static inline void stretch_end(void)
{
    scl_release();
}

// ─── Address lookup ────────────────────────────────────────────────────────────

static i2c_bb_addr_entry_t *find_addr(uint8_t addr7)
{
    for (int i = 0; i < I2C_BB_MAX_ADDRESSES; i++) {
        if (s_addrs[i].active && s_addrs[i].addr == addr7) {
            return &s_addrs[i];
        }
    }
    return NULL;
}

// ─── Commit pending write data ─────────────────────────────────────────────────

static void commit_write_if_pending(void)
{
    if ((s_state == ST_WRITE_RX   ||
         s_state == ST_WRITE_ACK  ||
         s_state == ST_WRITE_POST_ACK) && s_rx_len > 0) {
        i2c_bb_addr_entry_t *e = find_addr(s_matched_addr);
        if (e && e->on_write) {
            e->on_write(s_matched_addr, s_rx_buf, s_rx_len);
            i2c_bb_slave_stats.writes++;
        }
    }
}

// ─── Bus reset ─────────────────────────────────────────────────────────────────

/**
 * Release both bus lines and return the state machine to ST_IDLE.
 * Arms SDA SENSE_LOW for next-START detection; disables SCL SENSE.
 *
 * Does NOT commit pending write data — callers that need it (handle_stop,
 * timeout recovery) call commit_write_if_pending() themselves beforehand.
 */
static void reset_to_idle(void)
{
    sda_release();
    scl_release();
    s_state        = ST_IDLE;
    s_rx_len       = 0;
    s_matched_addr = 0;
    s_is_read      = false;
    pin_sense_disable(I2C_BB_SCL_PIN);
    pin_sense_set(I2C_BB_SDA_PIN, NRF_GPIO_PIN_SENSE_LOW);
}

// ─── START / STOP ──────────────────────────────────────────────────────────────

static void handle_start(void)
{
    commit_write_if_pending();
    i2c_bb_slave_stats.starts++;

    s_state        = ST_ADDR_RX;
    s_shift_reg    = 0;
    s_bit_cnt      = 8;
    s_rx_len       = 0;
    s_tx_idx       = 0;
    s_tx_len       = 0;
    s_matched_addr = 0;
    s_is_read      = false;

    /* SCL and SDA SENSE are re-armed centrally at the end of the IRQ handler
     * using the pre-read pin levels (scl_last / sda_last). */
}

static void handle_stop(void)
{
    commit_write_if_pending();
    i2c_bb_slave_stats.stops++;
    reset_to_idle();
}

// ─── SCL rising: SAMPLE SDA ────────────────────────────────────────────────────

static void handle_scl_rising(void)
{
    /*
     * SCL just went high.  SDA is now stable (master or us has set it up
     * during the preceding low phase).  Sample SDA first, then re-arm SDA
     * SENSE so any subsequent SDA change while SCL remains high is caught
     * as a START or STOP condition.
     */

    /* Record SCL period (DWT ticks between consecutive rising edges). */
    uint32_t now    = DWT->CYCCNT;
    uint32_t period = now - s_last_scl_rise;
    s_last_scl_rise = now;
    uint8_t idx = i2c_bb_slave_stats.scl_period_idx;
    i2c_bb_slave_stats.scl_periods[idx] = period;
    i2c_bb_slave_stats.scl_period_idx   = (idx + 1) & (I2C_BB_SCL_PERIOD_LOG - 1);

    switch (s_state) {

    case ST_ADDR_RX: {
        s_shift_reg = (s_shift_reg << 1) | (sda_read() ? 1u : 0u);
        s_bit_cnt--;
        if (s_bit_cnt == 0) {
            s_state = ST_ADDR_ACK;
        }
        /* SCL and SDA SENSE re-armed centrally at end of IRQ handler. */
        break;
    }

    case ST_WRITE_RX: {
        s_shift_reg = (s_shift_reg << 1) | (sda_read() ? 1u : 0u);
        s_bit_cnt--;
        if (s_bit_cnt == 0) {
            s_state = ST_WRITE_ACK;
        }
        /* SCL and SDA SENSE re-armed centrally at end of IRQ handler. */
        break;
    }

    case ST_WRITE_POST_ACK: {
        /*
         * Master is sampling our ACK/NACK (SCL just went high).
         * We do nothing here — SDA is still being driven by us (ACK low)
         * or released (NACK).  Wait for SCL to fall; only then does the
         * master release SDA and set up the first bit of the next byte.
         * SCL SENSE (SENSE_LOW) re-armed at end of IRQ handler.
         */
        s_state = ST_WRITE_RX;
        break;
    }

    case ST_READ_LAST_BIT: {
        /*
         * We are on the rising edge of the 8th data bit.
         * Master is sampling it RIGHT NOW — do NOT release SDA here.
         * Releasing SDA in this IRQ could corrupt the master's sample if the
         * handler runs before the master's setup-time has elapsed.
         * Instead we move to ST_READ_ACK_WAIT and release SDA on the very
         * next falling edge (in handle_scl_falling), once the master has
         * safely latched the bit.  SCL re-armed at end of handler.
         */
        s_state = ST_READ_ACK_WAIT;
        break;
    }

    case ST_READ_ACK_RX: {
        bool master_ack = !sda_read();

        if (master_ack && s_tx_idx < s_tx_len) {
            s_shift_reg = s_tx_buf[s_tx_idx++];
            s_bit_cnt   = 8;
            s_state     = ST_READ_TX;
            /* SCL re-armed at end of handler. */
        } else {
            reset_to_idle(); /* NACK or no more data — release bus, watch for STOP */
        }
        break;
    }

    default:
        break;
    }
}

// ─── SCL falling: CHANGE SDA ───────────────────────────────────────────────────

static void handle_scl_falling(void)
{
    /*
     * stretch_begin() pulls SCL low AND disables SDA SENSE.
     * Any SDA changes we make below are therefore invisible to the IRQ.
     */
    stretch_begin();

    switch (s_state) {

    /* ── Drive ACK/NACK for address byte ─────────────────────────────── */
    case ST_ADDR_ACK: {
        uint8_t addr7 = s_shift_reg >> 1;
        s_is_read     = (s_shift_reg & 0x01) != 0;

        i2c_bb_addr_entry_t *e = find_addr(addr7);
        if (!e) {
            i2c_bb_slave_stats.addr_misses++;
            reset_to_idle();
            return;
        }

        i2c_bb_slave_stats.addr_matches++;
        s_matched_addr = addr7;
        sda_low(); /* ACK */

        if (s_is_read) {
            s_tx_len = e->on_read ? e->on_read(addr7, s_tx_buf, I2C_BB_MAX_BUF) : 0;
            i2c_bb_slave_stats.reads++;
            if (s_tx_len > I2C_BB_MAX_BUF) s_tx_len = I2C_BB_MAX_BUF;
            s_tx_idx    = 0;
            s_shift_reg = (s_tx_len > 0) ? s_tx_buf[s_tx_idx++] : 0xFF;
            s_bit_cnt   = 8;
            s_state     = ST_READ_TX;
        } else {
            s_shift_reg = 0;
            s_bit_cnt   = 8;
            s_state     = ST_WRITE_POST_ACK;
        }
        stretch_end();
        return;
    }

    /* ── Post-ACK falling: master sets first bit of next data byte ──── */
    case ST_WRITE_RX: {
        /*
         * SCL just fell after the ACK slot.  Master is now setting up the
         * first bit of the next data byte (or a Repeated START / STOP will
         * follow).  Release our ACK; SCL re-armed at end of handler.
         */
        sda_release();
        stretch_end();
        return;
    }

    /* ── Drive ACK (or NACK on overflow) for received data byte ─────── */
    case ST_WRITE_ACK: {
        if (s_rx_len < I2C_BB_MAX_BUF) {
            s_rx_buf[s_rx_len++] = s_shift_reg;
            sda_low(); /* ACK */
            i2c_bb_slave_stats.acks_sent++;
        } else {
            sda_release(); /* NACK — buffer full */
        }
        s_shift_reg = 0;
        s_bit_cnt   = 8;
        s_state     = ST_WRITE_POST_ACK;
        stretch_end();
        return;
    }

    /* ── ACK low-phase: master is setting ACK/NACK on SDA ───────────── */
    case ST_READ_ACK_WAIT: {
        /*
         * SCL just fell after the 8th data bit.  The master has now safely
         * latched the bit — release SDA here so the master can drive ACK/NACK.
         * (sda_release was intentionally deferred from the rising edge to
         * guarantee the master's hold time on the last data bit.)
         */
        sda_release();
        s_state = ST_READ_ACK_RX;
        stretch_end();
        return;
    }

    /* ── Drive next TX data bit ──────────────────────────────────────── */
    case ST_READ_TX: {
        if (s_shift_reg & 0x80) {
            sda_release();
        } else {
            sda_low();
        }
        s_shift_reg <<= 1;
        s_bit_cnt--;

        if (s_bit_cnt == 0) {
            s_state = ST_READ_LAST_BIT;
        }
        stretch_end();
        return;
    }

    default:
        scl_release();
        break;
    }
}

// ─── GPIOTE PORT IRQ ───────────────────────────────────────────────────────────

void i2c_bb_slave_gpiote_irq_handler(void)
{
    if (!NRF_GPIOTE->EVENTS_PORT) {
        return;
    }

    /* ── Anomaly 119 workaround ───────────────────────────────────────────────
     *
     * Read LATCH, then DISABLE SENSE before clearing LATCH.
     * Clearing LATCH while SENSE is active causes DETECT to pulse 0→1
     * (pin still at sense level) → EVENTS_PORT immediately re-fires.
     */
    uint32_t latch0 = NRF_P0->LATCH;
    uint32_t latch1 = NRF_P1->LATCH;

    pin_sense_disable(I2C_BB_SDA_PIN);
    pin_sense_disable(I2C_BB_SCL_PIN);

    NRF_P0->LATCH           = latch0;
    NRF_P1->LATCH           = latch1;
    NRF_GPIOTE->EVENTS_PORT = 0;

    /* ── Read pin state ONCE, before state machine ────────────────────────────
     *
     * scl_last / sda_last capture the actual bus levels at the moment the
     * event was detected.  Reading here — after LATCH clear, before any
     * state-machine code — is the single authoritative pin read per IRQ.
     * The re-arm block at the bottom uses these values, avoiding any race
     * with scl_release() / sda_release() called inside the state machine.
     */
    s_scl_last = scl_read();
    s_sda_last = sda_read();

    uint32_t scl_latch = (I2C_BB_SCL_PIN < 32u) ? latch0 : latch1;
    uint32_t sda_latch = (I2C_BB_SDA_PIN < 32u) ? latch0 : latch1;

    bool scl_latched = (scl_latch & (1u << _PIN_BIT(I2C_BB_SCL_PIN))) != 0;
    bool sda_latched = (sda_latch & (1u << _PIN_BIT(I2C_BB_SDA_PIN))) != 0;

    uint32_t now_cyc = DWT->CYCCNT;

    /* ── Watchdog reset on real bus activity ──────────────────────────────────
     *
     * The FreeRTOS tickless-idle port (port_cmsis_systick.c) compensates
     * DWT->CYCCNT after sleep by adding the estimated elapsed cycles.  This
     * compensation runs AFTER sd_app_evt_wait() returns, i.e. after the first
     * GPIOTE ISR (START condition) has already run with the pre-sleep DWT value.
     * The next ISR (first SCL edge) therefore sees a DWT value that is hundreds
     * of millions of cycles higher than s_last_state_change_cyc, which would
     * trip the timeout on every I2C transaction started after a sleep period.
     *
     * Fix: reset the watchdog baseline whenever real I2C pin activity is
     * present (scl_latched or sda_latched).  The timeout is only meaningful for
     * a truly stuck bus where no edges arrive — exactly the case covered by the
     * remaining guard (no latch bits set, spurious ISR).
     */
    if (scl_latched || sda_latched) {
        s_last_state_change_cyc = now_cyc;
    }

    /* ── Timeout check ────────────────────────────────────────────────────────
     *
     * If the state machine has not made progress for I2C_BB_TIMEOUT_US µs
     * while not IDLE, the bus is assumed stuck and is forcibly reset.
     * Only checked when no latch bits are set (spurious ISR); when real edges
     * are present the watchdog was just reset above.
     */
    if (s_state != ST_IDLE) {
        if ((uint32_t)(now_cyc - s_last_state_change_cyc) > I2C_BB_TIMEOUT_CYCLES) {
            i2c_bb_slave_stats.scl_timeouts++;
            commit_write_if_pending();
            reset_to_idle();
            s_last_state_change_cyc = now_cyc;
            return;
        }
    }

    /* ── Dispatch ─────────────────────────────────────────────────────────────
     *
     * SDA before SCL: START/STOP takes priority — but only when SCL did not
     * also latch (simultaneous SCL + SDA latch means SDA changed during the
     * SCL-low phase, not while SCL was high → treat as a normal data clock).
     * A SDA latch while SCL is LOW is stale (SDA SENSE only armed while SCL
     * is high), so it is also ignored here.
     *
     * s_last_state_change_cyc is updated here, not on pin latch, so spurious
     * IRQs do not reset the stuck-bus watchdog.
     */
    if (sda_latched && !scl_latched && s_scl_last) {
        s_last_state_change_cyc = now_cyc;
        if (!s_sda_last) {
            handle_start(); /* SDA fell while SCL high → START / Repeated START */
        } else {
            handle_stop();  /* SDA rose while SCL high → STOP                   */
        }
        /* handle_stop calls reset_to_idle → s_state = ST_IDLE; re-arm skipped.
         * handle_start → s_state = ST_ADDR_RX; re-arm block below handles SENSE. */
    } else if (scl_latched) {
        s_last_state_change_cyc = now_cyc;
        if (s_scl_last) {
            handle_scl_rising();
        } else {
            handle_scl_falling();
        }
    }
    /* else: spurious — no state-machine function ran; re-arm below restores SENSE. */

    if (s_state != ST_IDLE) {
		scl_sense_arm(s_scl_last);
		sda_sense_arm(s_sda_last);
    }
    else {
        pin_sense_disable(I2C_BB_SCL_PIN);
        pin_sense_set(I2C_BB_SDA_PIN, NRF_GPIO_PIN_SENSE_LOW);
    }
}

// ─── Public API ────────────────────────────────────────────────────────────────

void i2c_bb_slave_init(void)
{
    memset(s_addrs, 0, sizeof(s_addrs));
    memset((void *)&i2c_bb_slave_stats, 0, sizeof(i2c_bb_slave_stats));
    s_state        = ST_IDLE;

    /* Enable DWT cycle counter for SCL period measurement. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT      = 0;
    DWT->CTRL       |= DWT_CTRL_CYCCNTENA_Msk;
    s_last_scl_rise         = 0;
    s_last_state_change_cyc = DWT->CYCCNT;
    s_matched_addr          = 0;
    s_is_read             = false;

    nrf_gpio_pin_clear(I2C_BB_SDA_PIN);
    nrf_gpio_pin_clear(I2C_BB_SCL_PIN);
    nrf_gpio_cfg_input(I2C_BB_SDA_PIN, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(I2C_BB_SCL_PIN, NRF_GPIO_PIN_PULLUP);

    /* Read actual bus levels and arm SDA SENSE for the next SDA transition.
     * SCL SENSE stays disabled in IDLE — SCL is only armed once a START is seen. */
    s_scl_last = scl_read();
    s_sda_last = sda_read();
    sda_sense_arm(s_sda_last);
    scl_sense_arm(s_scl_last);

    /* Clear stale LATCH bits for both pins, then clear EVENTS_PORT before
     * enabling the peripheral interrupt.  If SDA was already LOW when SENSE
     * was armed, the hardware sets the LATCH immediately (DETECT 0→1);
     * pin_sense_set() clears it, but EVENTS_PORT may already be set.
     * EVENTS_PORT is driven by the LATCH register — clearing the LATCHes
     * first ensures EVENTS_PORT stays clear after we write it. */
    _PIN_PORT(I2C_BB_SDA_PIN)->LATCH = (1u << _PIN_BIT(I2C_BB_SDA_PIN));
    _PIN_PORT(I2C_BB_SCL_PIN)->LATCH = (1u << _PIN_BIT(I2C_BB_SCL_PIN));
    NRF_GPIOTE->EVENTS_PORT          = 0;

    /* Enable the PORT event in the GPIOTE peripheral.
     * The NVIC for GPIOTE_IRQn is already enabled by the Arduino runtime. */

    NVIC_DisableIRQ(GPIOTE_IRQn);
    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    NVIC_SetPriority(GPIOTE_IRQn, I2C_BB_IRQ_PRIORITY);
    NVIC_EnableIRQ(GPIOTE_IRQn);


    nrf_gpiote_int_enable(NRF_GPIOTE, NRF_GPIOTE_INT_PORT_MASK);

}

bool i2c_bb_slave_register(uint8_t           addr,
                            i2c_bb_write_cb_t on_write,
                            i2c_bb_read_cb_t  on_read)
{
    for (int i = 0; i < I2C_BB_MAX_ADDRESSES; i++) {
        if (s_addrs[i].active && s_addrs[i].addr == addr) {
            s_addrs[i].on_write = on_write;
            s_addrs[i].on_read  = on_read;
            return true;
        }
    }
    for (int i = 0; i < I2C_BB_MAX_ADDRESSES; i++) {
        if (!s_addrs[i].active) {
            s_addrs[i].addr     = addr;
            s_addrs[i].active   = true;
            s_addrs[i].on_write = on_write;
            s_addrs[i].on_read  = on_read;
            return true;
        }
    }
    return false;
}

void i2c_bb_slave_unregister(uint8_t addr)
{
    for (int i = 0; i < I2C_BB_MAX_ADDRESSES; i++) {
        if (s_addrs[i].active && s_addrs[i].addr == addr) {
            memset(&s_addrs[i], 0, sizeof(s_addrs[i]));
            return;
        }
    }
}

#ifndef I2C_BB_OWN_IRQ
void GPIOTE_IRQHandler(void)
{
    i2c_bb_slave_gpiote_irq_handler();
}
#endif /* I2C_BB_OWN_IRQ */

#endif /* ALMEMO_SENSOR_RECEIVER */
