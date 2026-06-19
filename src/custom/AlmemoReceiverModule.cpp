#if defined(ALMEMO_SENSOR_RECEIVER)
#include "AlmemoReceiverModule.h"
#include "configuration.h"

extern "C" {
#include "i2c_bb_slave.h"
}

#include <nrf.h>      /* NRF_GPIOTE */
#include <stdint.h>   /* UINT32_MAX */
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

/**
 * Encode a signed power-of-ten exponent into the ALMEMO info-block byte [1]:
 * high nibble = sign(bit3) | magnitude(bits2-0), low nibble = 0.
 * E.g. exponent -2 → (0x8|0x2)<<4 = 0xA0 (2 decimal places). Mirrors the
 * decode in AlmemoI2CSensor::parseSlot().
 */
static uint8_t encodeExponentByte(int8_t exponent)
{
    uint8_t mag  = (uint8_t)(exponent < 0 ? -exponent : exponent) & 0x07;
    uint8_t sign = (exponent < 0) ? 0x08 : 0x00;
    return (uint8_t)((sign | mag) << 4);
}

/**
 * Fill a DIGITAL_SENSOR_INFO_SIZE-byte sensor info block (digital, type 0x37)
 * in the virtual EEPROM from unit + exponent + comment carried in the packet.
 *
 *   [0]     SensorTyp  (0x37)
 *   [1]     exponent byte (see encodeExponentByte)
 *   [7]     bitmask 0xFF
 *   [30-31] Einheit (2 bytes, straight from the sensor)
 *   [32-41] Kommentar (up to 10 bytes)
 *   [59]    block length (60)
 */
static void buildSensorInfo(uint8_t *buf, const char unit[2], int8_t exponent, const char *comment)
{
    memset(buf, 0, DIGITAL_SENSOR_INFO_SIZE);
    buf[0]  = 0x37;
    buf[1]  = encodeExponentByte(exponent);
    buf[7]  = 0xFF;
    buf[30] = (uint8_t)unit[0];
    buf[31] = (uint8_t)unit[1];
    if (comment) {
        /* Field is 10 bytes, buf already zeroed; strncpy null-pads and never
         * writes past 10 even if comment is longer. */
        strncpy((char *)&buf[32], comment, 10);
    }
    buf[59] = DIGITAL_SENSOR_INFO_SIZE;
}

/** Empty-slot info block: SensorTyp 0xFF, everything else zeroed. */
static void buildEmptyInfo(uint8_t *buf)
{
    memset(buf, 0, DIGITAL_SENSOR_INFO_SIZE);
    buf[0] = 0xFF;
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
    if (NRF_GPIOTE->EVENTS_PORT)
    {
        i2c_bb_slave_gpiote_irq_handler();
    }

    /* Dispatch EVENTS_IN callbacks (radio DIO1, EXT_CHRG_DETECT, …).
     * Skip the call entirely when no IN channels are armed — common case
     * inside this build, since PORT (I2C slave) is the hot path.  Spares
     * the function call + the 8-channel scan inside it. */
    if ((NRF_GPIOTE->INTENSET & 0xFFu) != 0) {
        gpiote_dispatch_events_in();
    }

    #if __CORTEX_M == 0x04
      __DSB(); __NOP();__NOP();__NOP();__NOP();
    #endif
}

/* -------------------------------------------------------------------------
 * writeEeprom — EEPROM write from task context
 *
 * No NVIC_DisableIRQ around the memcpy: masking GPIOTE for the duration of a
 * write blocks SCL-edge IRQs and desyncs the bit-bang state machine mid-
 * transaction.  A torn read from the ISR is harmless — the master will get one
 * transaction with a half-updated value and the next read returns the fresh one.
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

    memcpy(&eeprom[idx][offset], data, len);
}

/* -------------------------------------------------------------------------
 * writeReg — register write from task context
 *
 * No NVIC_DisableIRQ — see writeEeprom rationale.
 * ---------------------------------------------------------------------- */

void I2CSlaveThread::writeReg(uint8_t reg, const uint8_t data[REG_MAX])
{
    if (reg >= REG_COUNT)
        return;

    memcpy(reg_data[reg], data, REG_MAX);
    reg_size[reg] = REG_MAX;
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
 * EEPROM slot layout (sensor info blocks live in eeprom[0])
 * ---------------------------------------------------------------------- */

static constexpr uint8_t SLOT_OFFSET[I2CSlaveThread::MAX_NODES] = {
    0x08, 0x44, 0x80, 0xBC
};

/* "No value" pattern returned for empty slots on register reads from 0x40. */
static const uint8_t ALMEMO_NO_VAL[I2CSlaveThread::REG_MAX] = { 0x00, 0x80, 0x00, 0x00 };

/* -------------------------------------------------------------------------
 * Constructor
 * ---------------------------------------------------------------------- */

I2CSlaveThread::I2CSlaveThread() : OSThread("I2CSlave")
{
    s_instance = this;

    memset(eeprom,   0, sizeof(eeprom));
    memset(reg_data, 0, sizeof(reg_data));
    memset(reg_size, 0, sizeof(reg_size));

    /* EEPROM 0 (0x50): device identity at start, firmware version at end. */
    memcpy(&eeprom[0][0x00], "FHAD46  ", 8);
    strncpy((char*)&eeprom[0][0xF8], "   6.66", 8);

    /* All slots start empty; first packet from each node populates one. */
    for (uint8_t i = 0; i < MAX_NODES; i++)
        clearSlot(i);

    /* Init GPIO, arm PORT SENSE, enable GPIOTE interrupt. */
    i2c_bb_slave_init();

    /* TinyUSB sets USBD_IRQn to prio 2 (same as our GPIOTE) — equal-priority
     * ISRs do not preempt each other, so a USBD burst (log flush, SOF storm)
     * can delay PORT-event handling enough to skew the bit-bang state machine.
     * Drop USBD to prio 3 so GPIOTE@2 preempts it. */
    NVIC_SetPriority(USBD_IRQn, 3);

    /* Prepare blue LED (P0.06) for ISR debug (LED_STATE_ON=0, common anode). */
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 6));
    nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(0, 6)); /* start OFF */

    i2c_bb_slave_register(0x50, onWrite, onRead);
    i2c_bb_slave_register(0x51, onWrite, onRead);
    i2c_bb_slave_register(0x40, onWrite, onRead);
}

/* -------------------------------------------------------------------------
 * Slot helpers
 *
 * onValue() (Router task) and runOnce() (I2CSlave OSThread task) both touch
 * nodes[]. Both are FreeRTOS tasks running at low frequency (one packet per
 * sender interval, runOnce every ~1 s); a torn read of a 32-bit field is
 * atomic on Cortex-M4 and the worst observable race — runOnce missing or
 * mis-expiring once — is self-correcting on the next tick. No mutex needed.
 *
 * The unregister/register calls inside muteI2CFor/unmuteI2C modify the
 * i2c_bb_slave callback table read from the GPIOTE ISR; those are wrapped in
 * NVIC_DisableIRQ(GPIOTE_IRQn) for ISR-vs-task atomicity.
 * ---------------------------------------------------------------------- */

uint8_t I2CSlaveThread::findOrAllocSlot(uint32_t node_id, uint8_t sub)
{
    for (uint8_t i = 0; i < MAX_NODES; i++)
        if (nodes[i].in_use && nodes[i].node_id == node_id && nodes[i].sub == sub) return i;
    for (uint8_t i = 0; i < MAX_NODES; i++)
        if (!nodes[i].in_use) return i;
    /* All occupied: evict the LRU node (oldest last_seen_ms). */
    uint8_t oldest = 0;
    for (uint8_t i = 1; i < MAX_NODES; i++)
        if (nodes[i].last_seen_ms < nodes[oldest].last_seen_ms) oldest = i;
    expireSlot(oldest);
    return oldest;
}

void I2CSlaveThread::initSlotForNode(uint8_t slot, uint32_t node_id, uint8_t sub, const char unit[2], int8_t exponent)
{
    /* Kommentar = "NNNN.S": last 4 hex chars of node_id (short name) + sub index,
     * so several values from the same sender stay distinguishable. */
    char comment[11];
    snprintf(comment, sizeof(comment), "%04X.%u", (unsigned)(node_id & 0xFFFF), sub);

    uint8_t buf[DIGITAL_SENSOR_INFO_SIZE];
    buildSensorInfo(buf, unit, exponent, comment);
    writeEeprom(0x50, SLOT_OFFSET[slot], buf, DIGITAL_SENSOR_INFO_SIZE);
}

void I2CSlaveThread::clearSlot(uint8_t slot)
{
    uint8_t buf[DIGITAL_SENSOR_INFO_SIZE];
    buildEmptyInfo(buf);
    writeEeprom(0x50, SLOT_OFFSET[slot], buf, DIGITAL_SENSOR_INFO_SIZE);
    writeReg(slot, ALMEMO_NO_VAL);
}

void I2CSlaveThread::writeRawForSlot(uint8_t slot, int16_t raw)
{
    uint8_t data[REG_MAX] = {
        0x00, 0x40,
        (uint8_t)((uint16_t)raw >> 8),
        (uint8_t)((uint16_t)raw & 0xFF),
    };
    writeReg(slot, data);
}

void I2CSlaveThread::expireSlot(uint8_t slot)
{
    LOG_INFO("AlmemoRx: slot %u expired (node=%08X sub=%u)", slot, (unsigned)nodes[slot].node_id, nodes[slot].sub);
    nodes[slot] = NodeEntry{};
    clearSlot(slot);
    muteI2CFor(ALMEMO_RX_MUTE_MS);
}

void I2CSlaveThread::muteI2CFor(uint32_t ms)
{
    /* Idempotent: extend the deadline; unregister only on first entry into the
     * window. unregister() is itself idempotent against missing entries. */
    if (!muteUntilMs) {
        NVIC_DisableIRQ(GPIOTE_IRQn);
        i2c_bb_slave_unregister(0x40);
        i2c_bb_slave_unregister(0x50);
        i2c_bb_slave_unregister(0x51);
        NVIC_EnableIRQ(GPIOTE_IRQn);
        LOG_INFO("I2CSlave: muted for %lums (addresses unregistered)", (unsigned long)ms);
    }
    muteUntilMs = millis() + ms;
    if (!muteUntilMs) muteUntilMs = 1; /* reserve 0 as "not muted" sentinel */
}

void I2CSlaveThread::unmuteI2C()
{
    NVIC_DisableIRQ(GPIOTE_IRQn);
    i2c_bb_slave_register(0x50, onWrite, onRead);
    i2c_bb_slave_register(0x51, onWrite, onRead);
    i2c_bb_slave_register(0x40, onWrite, onRead);
    NVIC_EnableIRQ(GPIOTE_IRQn);
    muteUntilMs = 0;
    LOG_INFO("I2CSlave: un-muted, addresses re-registered");
}

/* -------------------------------------------------------------------------
 * onValue — mesh -> slot table update (called from Router task)
 * ---------------------------------------------------------------------- */

void I2CSlaveThread::onValue(uint32_t node_id, uint8_t sub, const char unit[2], int8_t exponent, int16_t raw)
{
    uint8_t slot = findOrAllocSlot(node_id, sub);

    uint32_t now = millis();
    NodeEntry &n = nodes[slot];
    if (!n.in_use || n.node_id != node_id || n.sub != sub) {
        /* New stream (fresh slot or LRU eviction of a different one). */
        n.in_use       = true;
        n.node_id      = node_id;
        n.sub          = sub;
        n.prev_seen_ms = 0;
        n.last_seen_ms = now;
        initSlotForNode(slot, node_id, sub, unit, exponent);
        /* Value count just increased — force the master to rescan EEPROM by
         * going silent on I2C for ALMEMO_RX_MUTE_MS. Same mechanism we use on
         * expiration; expireSlot() already does this for the eviction case. */
        muteI2CFor(ALMEMO_RX_MUTE_MS);
        LOG_INFO("AlmemoRx: slot %u allocated for node=%08X sub=%u", slot, (unsigned)node_id, sub);
    } else {
        n.prev_seen_ms = n.last_seen_ms;
        n.last_seen_ms = now;
    }

    writeRawForSlot(slot, raw);
}

/* -------------------------------------------------------------------------
 * OSThread runOnce — periodic expiration scan + mute end + stats log
 * ---------------------------------------------------------------------- */

int32_t I2CSlaveThread::runOnce()
{
    /* End the mute window? Re-register addresses so the master sees us again. */
    if (muteUntilMs && (int32_t)(millis() - muteUntilMs) >= 0)
        unmuteI2C();

    /* Expiration scan. */
    {
        uint32_t now = millis();
        for (uint8_t i = 0; i < MAX_NODES; i++) {
            NodeEntry &n = nodes[i];
            if (!n.in_use) continue;
            uint32_t timeout;
            if (n.prev_seen_ms == 0) {
                timeout = ALMEMO_RX_FIRST_PACKET_TIMEOUT_MS;
            } else {
                uint32_t interval = n.last_seen_ms - n.prev_seen_ms;
                timeout = interval + interval / 5 + 3000; /* +20% +3s margin (mesh jitter / retries) */
            }
            if ((int32_t)(now - n.last_seen_ms) > (int32_t)timeout)
                expireSlot(i);
        }
    }

    /* Stats log, throttled to ~5s independently of the runOnce cadence.
     * Single concise line for the common case.  Anomalies (timeouts, addr
     * misses, spurious IRQs, wr/rd/ack mismatch) are logged separately as
     * persistent WARN lines whenever their counter is non-zero or the
     * invariant is broken — so they're easy to grep for, and you never have
     * to hunt through the rolling log for the one bad entry. */
    static uint32_t lastLogMs = 0;
    if ((int32_t)(millis() - lastLogMs) >= 5000) {
        lastLogMs = millis();
        uint32_t starts  = i2c_bb_slave_stats.starts;
        uint32_t stops   = i2c_bb_slave_stats.stops;
        uint32_t matches = i2c_bb_slave_stats.addr_matches;
        uint32_t misses  = i2c_bb_slave_stats.addr_misses;
        uint32_t writes  = i2c_bb_slave_stats.writes;
        uint32_t reads   = i2c_bb_slave_stats.reads;
        uint32_t acks    = i2c_bb_slave_stats.acks_sent;
        uint32_t bus_to  = i2c_bb_slave_stats.scl_timeouts;
        uint32_t a119    = i2c_bb_slave_stats.anomaly119_irqs;
        uint32_t wr_edge = i2c_bb_slave_stats.write_edge_loss;
        uint32_t rd_edge = i2c_bb_slave_stats.read_edge_loss;
        /* Moving-window min/avg/max over the last I2C_BB_IRQ_DUR_LOG IRQ
         * durations.  Empty ring slots (before the ring is filled) are skipped
         * so the window doesn't bias the stats at startup. */
        uint32_t irq_min_cyc = UINT32_MAX;
        uint32_t irq_max_cyc = 0;
        uint32_t irq_sum     = 0;
        uint8_t  irq_n       = 0;
        for (uint8_t i = 0; i < I2C_BB_IRQ_DUR_LOG; i++) {
            uint32_t v = i2c_bb_slave_stats.irq_dur_cyc[i];
            if (v == 0) continue;
            if (v < irq_min_cyc) irq_min_cyc = v;
            if (v > irq_max_cyc) irq_max_cyc = v;
            irq_sum += v;
            irq_n++;
        }
        /* 64 MHz core → 64 cycles per µs. */
        uint32_t irq_min_us = irq_n ? irq_min_cyc / 64 : 0;
        uint32_t irq_max_us = irq_max_cyc / 64;
        uint32_t irq_avg_us = irq_n ? (irq_sum / irq_n / 64) : 0;

        LOG_INFO("I2CSlave: start=%lu stop=%lu match=%lu miss=%lu state=%s irq_us=%lu/%lu/%lu (min/avg/max)",
                 starts, stops, matches, misses,
                 i2c_bb_slave_state_name(),
                 (unsigned long)irq_min_us,
                 (unsigned long)irq_avg_us,
                 (unsigned long)irq_max_us);

        if (bus_to > 0) {
            LOG_WARN("I2CSlave bus_timeout: %lu (state machine forced to IDLE in IRQ watchdog)", bus_to);
        }
        if (a119 > 0) {
            LOG_WARN("I2CSlave anomaly119: %lu (PORT event with no matching latch)", a119);
        }
        if (wr_edge > 0) {
            LOG_WARN("I2CSlave write_edge_loss: %lu (ST_WRITE_POST_ACK falling-edge recovery)", wr_edge);
        }
        if (rd_edge > 0) {
            LOG_WARN("I2CSlave read_edge_loss: %lu (ST_READ_LAST_BIT falling-edge recovery)", rd_edge);
        }
        /* wr/rd/ack invariant: each polling cycle is W-then-R; ack ≈ wr (one
         * ACK per write byte, ±1 for an in-flight transaction at log time). */
        int32_t ack_skew = (int32_t)acks - (int32_t)writes;
        if (writes != reads || ack_skew < -1 || ack_skew > 1) {
            LOG_WARN("I2CSlave wr/rd/ack drift: wr=%lu rd=%lu ack=%lu", writes, reads, acks);
        }
    }

    /* Stuck-state debug: whenever the state machine sits in a non-IDLE state
     * for more than 1s, log which state plus the bit/byte counters and the
     * IRQ-entries delta since we first noticed.  irq_delta == 0 → no IRQs are
     * firing at all (bus genuinely quiet), so we just need a master poke to
     * recover.  irq_delta > 0 → IRQs ARE firing but state is not progressing,
     * i.e. a real state-machine bug. */
    static uint32_t stuckSinceMs    = 0;
    static uint32_t stuckIrqAtStart = 0;
    if (i2c_bb_slave_is_idle()) {
        stuckSinceMs    = 0;
        stuckIrqAtStart = 0;
    } else if (stuckSinceMs == 0) {
        stuckSinceMs    = millis();
        stuckIrqAtStart = i2c_bb_slave_stats.irq_entries;
    } else if ((int32_t)(millis() - stuckSinceMs) > 1000) {
        uint8_t  bit_cnt = 0, rx_len = 0, tx_idx = 0;
        i2c_bb_slave_get_debug(&bit_cnt, &rx_len, &tx_idx);
        uint32_t irq_delta = i2c_bb_slave_stats.irq_entries - stuckIrqAtStart;
        LOG_WARN("I2CSlave stuck in %s for %ldms: bit_cnt=%u rx_len=%u tx_idx=%u irq_delta=%lu",
                 i2c_bb_slave_state_name(),
                 (long)(millis() - stuckSinceMs),
                 bit_cnt, rx_len, tx_idx,
                 (unsigned long)irq_delta);
    }

    /* While muted, schedule the next wakeup tightly on the deadline so the
     * re-register snaps in close to ALMEMO_RX_MUTE_MS rather than the 1Hz tick. */
    if (muteUntilMs) {
        int32_t remaining = (int32_t)(muteUntilMs - millis());
        if (remaining < 0) remaining = 0;
        return remaining + 10;
    }
    return 1000;
}

/* -------------------------------------------------------------------------
 * Write callback (called from ISR — keep short!)
 * ---------------------------------------------------------------------- */

void I2CSlaveThread::onWrite(uint8_t addr, const uint8_t *buf, uint8_t len)
{
    if (!s_instance || len == 0)
        return;


    if (addr == 0x50 || addr == 0x51) {
        /* Master is read-only on the EEPROM addresses — only legitimate write is
         * a single address-pointer byte.  Anything longer is bit-bang glitch
         * noise that would otherwise corrupt the EEPROM (e.g. invent sensor
         * slots).  Drop multi-byte writes entirely. */
        if (len != 1)
            return;
        uint8_t idx   = (addr == 0x51) ? 1 : 0;
        s_eeprom_ptr[idx] = buf[0];
#if 0
        /* Full write path — disabled while the master is read-only. Restore
         * when EEPROM writes from the master are needed again. */
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
#endif
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
