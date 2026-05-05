#if defined(ALMEMO_SENSOR_RECEIVER) && defined(I2C_SLAVE_USE_TWIS)

#include "i2c_twis_slave.h"

#include "nrf.h"
#include "nrf_gpio.h"
#include <string.h>

/* ─── Per-instance state ─────────────────────────────────────────────────── */

typedef struct {
    NRF_TWIS_Type *twis;
    IRQn_Type      irqn;

    uint8_t        addr0;
    uint8_t        addr1;         /* 0 = unused */
    bool           addr1_active;

    i2c_twis_write_cb_t cb0_write;
    i2c_twis_read_cb_t  cb0_read;
    i2c_twis_write_cb_t cb1_write;
    i2c_twis_read_cb_t  cb1_read;

    uint8_t        matched_addr;  /* last address matched in the current transaction */
    bool           pending_write; /* WRITE event seen; RX buffer is armed            */

    uint8_t        rx_buf[I2C_TWIS_RX_BUF_SIZE];
    uint8_t        tx_buf[I2C_TWIS_TX_BUF_SIZE];
} twis_state_t;

static twis_state_t s_twis0 = {
    .twis = NRF_TWIS0,
    .irqn = SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn,
};

static twis_state_t s_twis1 = {
    .twis = NRF_TWIS1,
    .irqn = SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn,
};

volatile i2c_twis_slave_stats_t i2c_twis_slave_stats;

/* ─── Helpers ────────────────────────────────────────────────────────────── */

static inline uint8_t matched_addr(twis_state_t *s)
{
    uint8_t slot = (uint8_t)(s->twis->MATCH & 0x1u);
    return (slot == 0) ? s->addr0 : s->addr1;
}

static inline i2c_twis_write_cb_t pick_write_cb(twis_state_t *s, uint8_t addr)
{
    if (addr == s->addr0) return s->cb0_write;
    if (s->addr1_active && addr == s->addr1) return s->cb1_write;
    return NULL;
}

static inline i2c_twis_read_cb_t pick_read_cb(twis_state_t *s, uint8_t addr)
{
    if (addr == s->addr0) return s->cb0_read;
    if (s->addr1_active && addr == s->addr1) return s->cb1_read;
    return NULL;
}

static void bump_match_stat(uint8_t addr)
{
    switch (addr) {
        case 0x50: i2c_twis_slave_stats.match_0x50++; break;
        case 0x51: i2c_twis_slave_stats.match_0x51++; break;
        case 0x40: i2c_twis_slave_stats.match_0x40++; break;
        default: break;
    }
}

/* Flush a completed write phase to its callback and reset the pending flag.
 * Called from both STOPPED (normal end) and READ (repeated-start read pattern). */
static void flush_pending_write(twis_state_t *s)
{
    if (!s->pending_write) return;

    uint32_t n = s->twis->RXD.AMOUNT;
    if (n > I2C_TWIS_RX_BUF_SIZE) n = I2C_TWIS_RX_BUF_SIZE;

    i2c_twis_write_cb_t cb = pick_write_cb(s, s->matched_addr);
    if (cb && n > 0) {
        cb(s->matched_addr, s->rx_buf, (uint8_t)n);
        i2c_twis_slave_stats.writes++;
    }
    s->pending_write = false;
}

/* ─── IRQ core ───────────────────────────────────────────────────────────── */

static void twis_irq(twis_state_t *s)
{
    NRF_TWIS_Type *twis = s->twis;

    /* WRITE: master wants to write to us — arm RX buffer. */
    if (twis->EVENTS_WRITE) {
        twis->EVENTS_WRITE = 0;

        s->matched_addr  = matched_addr(s);
        bump_match_stat(s->matched_addr);

        twis->RXD.PTR    = (uint32_t)s->rx_buf;
        twis->RXD.MAXCNT = I2C_TWIS_RX_BUF_SIZE;
        s->pending_write = true;
        twis->TASKS_PREPARERX = 1;
    }

    /* READ: master wants to read — flush any preceding write (repeated-start
     * register-read pattern), then fetch data from callback and arm TX. */
    if (twis->EVENTS_READ) {
        twis->EVENTS_READ = 0;

        flush_pending_write(s);

        s->matched_addr = matched_addr(s);
        bump_match_stat(s->matched_addr);

        i2c_twis_read_cb_t cb = pick_read_cb(s, s->matched_addr);
        uint8_t tx_len = 0;
        if (cb) {
            tx_len = cb(s->matched_addr, s->tx_buf, I2C_TWIS_TX_BUF_SIZE);
        }
        if (tx_len == 0) {
            /* No data to send — return bus-idle pattern.  Master will read 0xFF
             * until it NACKs.  TXD.MAXCNT=0 is legal but the master would clock
             * out undefined data from the SRAM pointer. */
            s->tx_buf[0] = 0xFF;
            tx_len = 1;
        }
        twis->TXD.PTR    = (uint32_t)s->tx_buf;
        twis->TXD.MAXCNT = tx_len;
        twis->TASKS_PREPARETX = 1;

        i2c_twis_slave_stats.reads++;
    }

    /* STOPPED: end of transaction.  Flush trailing write data. */
    if (twis->EVENTS_STOPPED) {
        twis->EVENTS_STOPPED = 0;
        flush_pending_write(s);
        i2c_twis_slave_stats.stopped++;
    }

    /* ERROR: log and clear ERRORSRC (write-1-to-clear). */
    if (twis->EVENTS_ERROR) {
        twis->EVENTS_ERROR = 0;
        uint32_t src = twis->ERRORSRC;
        twis->ERRORSRC = src; /* w1c */
        i2c_twis_slave_stats.last_errorsrc = src;
        i2c_twis_slave_stats.errors++;
        s->pending_write = false;
    }
}

/* ─── IRQ handlers (strong symbols, override startup.S weak aliases) ─────── */

void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler(void)
{
    twis_irq(&s_twis0);
}

void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler(void)
{
    twis_irq(&s_twis1);
}

/* ─── Setup / registration ───────────────────────────────────────────────── */

static void twis_setup(twis_state_t *s,
                       uint8_t sda_pin, uint8_t scl_pin,
                       uint8_t addr0,   uint8_t addr1)
{
    NRF_TWIS_Type *twis = s->twis;

    /* Ensure peripheral is disabled before touching PSEL / ADDRESS / CONFIG.
     * Writing Disabled turns off whichever alt-function (SPIM/SPIS/TWIM/TWIS)
     * might have been left enabled on this instance. */
    twis->ENABLE = TWIS_ENABLE_ENABLE_Disabled << TWIS_ENABLE_ENABLE_Pos;

    twis->PSEL.SDA = sda_pin;
    twis->PSEL.SCL = scl_pin;

    twis->ADDRESS[0] = addr0;
    s->addr0 = addr0;

    if (addr1 != 0) {
        twis->ADDRESS[1] = addr1;
        twis->CONFIG = (TWIS_CONFIG_ADDRESS0_Msk | TWIS_CONFIG_ADDRESS1_Msk);
        s->addr1        = addr1;
        s->addr1_active = true;
    } else {
        twis->CONFIG = TWIS_CONFIG_ADDRESS0_Msk;
        s->addr1        = 0;
        s->addr1_active = false;
    }

    twis->EVENTS_STOPPED = 0;
    twis->EVENTS_WRITE   = 0;
    twis->EVENTS_READ    = 0;
    twis->EVENTS_ERROR   = 0;

    twis->INTENSET = TWIS_INTEN_STOPPED_Msk | TWIS_INTEN_WRITE_Msk
                   | TWIS_INTEN_READ_Msk    | TWIS_INTEN_ERROR_Msk;

    twis->ENABLE = TWIS_ENABLE_ENABLE_Enabled << TWIS_ENABLE_ENABLE_Pos;

    NVIC_SetPriority(s->irqn, I2C_TWIS_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(s->irqn);
    NVIC_EnableIRQ(s->irqn);
}

void i2c_twis_slave_init(void)
{
    memset((void*)&i2c_twis_slave_stats, 0, sizeof(i2c_twis_slave_stats));

    /* TWIS0 handles 0x50 + 0x51, TWIS1 handles 0x40.
     * Both are pointed at the same SDA/SCL pins. */
    twis_setup(&s_twis0, I2C_TWIS_SDA_PIN, I2C_TWIS_SCL_PIN, 0x50, 0x51);
    twis_setup(&s_twis1, I2C_TWIS_SDA_PIN, I2C_TWIS_SCL_PIN, 0x40, 0);
}

bool i2c_twis_slave_register(uint8_t addr,
                              i2c_twis_write_cb_t on_write,
                              i2c_twis_read_cb_t  on_read)
{
    if (addr == s_twis0.addr0) {
        s_twis0.cb0_write = on_write;
        s_twis0.cb0_read  = on_read;
        return true;
    }
    if (s_twis0.addr1_active && addr == s_twis0.addr1) {
        s_twis0.cb1_write = on_write;
        s_twis0.cb1_read  = on_read;
        return true;
    }
    if (addr == s_twis1.addr0) {
        s_twis1.cb0_write = on_write;
        s_twis1.cb0_read  = on_read;
        return true;
    }
    return false;
}

#endif /* ALMEMO_SENSOR_RECEIVER && I2C_SLAVE_USE_TWIS */
