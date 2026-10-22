#if defined(ALMEMO_SENSOR_RECEIVER)
#include "AlmemoReceiverModule.h"
#include "AlmemoMenuSettings.h"
#include "mesh/MeshService.h" /* reloadConfig(), s. applyMenuSetting() */
#include "airtime.h"
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
 *   0x00–0x07  device name      ("LORA-GW ")
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

/**
 * True iff the packet value carries a temperature, i.e. its unit is "°C".
 * The degree sign arrives as 0xF8 from this project's SHT path; real ALMEMO
 * sensors may store a different degree byte, so the common 0xDF/0xB0 encodings
 * are accepted too. Adjust here if a sensor reports °C with another leading byte.
 */
static inline bool isTemperatureUnit(const char unit[2])
{
    if (unit[1] != 'C')
        return false;
    uint8_t deg = (uint8_t)unit[0];
    return deg == 0xF8 || deg == 0xDF || deg == 0xB0;
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
volatile uint16_t I2CSlaveThread::s_menu_ptr        = 0;
volatile bool     I2CSlaveThread::s_edit_pending    = false;
volatile uint8_t  I2CSlaveThread::s_edit_item       = 0;
volatile int32_t  I2CSlaveThread::s_edit_raw        = 0;
MenuWireHeader    I2CSlaveThread::s_menu_hdr        = {};
I2CSlaveThread   *I2CSlaveThread::s_instance        = nullptr;
volatile bool     I2CSlaveThread::s_write_pending   = false;
volatile uint8_t  I2CSlaveThread::s_write_addr      = 0;
volatile uint8_t  I2CSlaveThread::s_write_start     = 0;
volatile uint8_t  I2CSlaveThread::s_write_count     = 0;

/* -------------------------------------------------------------------------
 * EEPROM slot layout (sensor info blocks live in eeprom[0])
 * ---------------------------------------------------------------------- */

static constexpr uint8_t SLOT_OFFSET[I2CSlaveThread::MAX_SLOTS] = {
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

    /* EEPROM 0 (0x50): device identity at start, firmware version at end.
     * Both fields are 8 bytes, space-padded the ALMEMO way -- name left, version
     * right aligned.
     *
     * Die VERSION ist tragend, nicht Kosmetik: der Node leitet daraus die
     * Geraetegeneration ab (6.xx = D6, 7.xx = D7, s. AlmemoI2CSensor::begin()).
     * Auf 7.xx geaendert wuerde er uns als D7 abweisen und auf den D7-UART-Pfad
     * schicken, wo wir nicht antworten. Also bei 6.xx bleiben.
     *
     * Der NAME ist frei -- der Node nimmt jeden String. Ob die ALMEMO-Logger ihn
     * nur anzeigen oder gegen eine Typentabelle pruefen, ist unklar: vorher
     * stand hier "FHAD46  ", die Kennung eines echten Ahlborn-Fuehlers. Falls
     * ein Logger den Fuehler nicht mehr annimmt, ist das der erste Verdacht. */
    static_assert(sizeof(ALMEMO_GW_NAME) - 1 <= 8, "ALMEMO_GW_NAME passt nicht ins Namensfeld");
    memset(&eeprom[0][0x00], ' ', 8); /* ALMEMO-Namensfeld ist leerzeichengefuellt */
    memcpy(&eeprom[0][0x00], ALMEMO_GW_NAME, sizeof(ALMEMO_GW_NAME) - 1);
    strncpy((char*)&eeprom[0][0xF8], "   6.66", 8);

    /* Braucht Kennung und Version aus dem EEPROM, also NACH deren Initialisierung. */
    buildMenu();

    /* All slots start empty; first packet from each node populates one. */
    for (uint8_t i = 0; i < MAX_SLOTS; i++)
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
    /* Vierte und damit LETZTE freie Adresse: I2C_BB_MAX_ADDRESSES ist 4. Die
     * uebrigen Aufrufe ignorieren ihren Rueckgabewert, ein fuenfter Eintrag
     * wuerde also still scheitern -- hier deshalb laut. */
    if (!i2c_bb_slave_register(ALMEMO_MENU_I2C_ADDR, onWrite, onRead))
        LOG_ERROR("I2CSlave: Menue-Adresse 0x%02x nicht registriert, Tabelle voll (max %u)",
                  (unsigned)ALMEMO_MENU_I2C_ADDR, (unsigned)I2C_BB_MAX_ADDRESSES);
}

/* -------------------------------------------------------------------------
 * Slot helpers
 *
 * onValue() runs on the Router task and is the only writer of slots[]. The
 * table is first-come-first-served and never cleared at runtime, so there is
 * no task-vs-task race to guard against.
 * ---------------------------------------------------------------------- */

uint8_t I2CSlaveThread::findOrAllocSlot(uint32_t node_id, uint8_t sub)
{
    for (uint8_t i = 0; i < MAX_SLOTS; i++)
        if (slots[i].in_use && slots[i].node_id == node_id && slots[i].sub == sub) return i;
    for (uint8_t i = 0; i < MAX_SLOTS; i++)
        if (!slots[i].in_use) return i;
    /* All 4 channels already taken — ignore this new stream. */
    return 0xFF;
}

void I2CSlaveThread::initSlotForNode(uint8_t slot, uint32_t node_id, uint8_t sub, const char unit[2], int8_t exponent)
{
    /* Kommentar = node short name (last 4 hex of node_id) + decoded sub index.
     * sub packs (channel << 4) | valueIndex (see AlmemoValue::slot):
     *   channel 0  → loose single values (e.g. SHT)        → "NNNN.<v>"
     *   channel >0 → ALMEMO sensor, numbered from 1 in the   → "NNNN.M<c>.<v>"
     *                packet but shown 0-based here (channel-1).
     * Keeps several values from the same sender distinguishable. */
    const unsigned node16  = (unsigned)(node_id & 0xFFFF);
    const uint8_t  channel = almemoSlotChannel(sub);
    const uint8_t  valIdx  = almemoSlotValueIndex(sub);
    char comment[11];
    if (channel == 0)
        snprintf(comment, sizeof(comment), "%04X.%u", node16, valIdx);
    else
        snprintf(comment, sizeof(comment), "%04X.M%u.%u", node16, channel - 1, valIdx);

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

/* -------------------------------------------------------------------------
 * I2C mute — brief NACK window to make the master rescan the EEPROM.
 *
 * unregister()/register() modify the i2c_bb_slave callback table that the
 * GPIOTE ISR reads, so both are wrapped in NVIC_DisableIRQ(GPIOTE_IRQn) for
 * ISR-vs-task atomicity. Both are called ONLY from runOnce() (I2CSlave task),
 * so muteUntilMs and the registration state are single-task — no task-vs-task
 * race. A pending rescan (rescanRequested, set by onValue) drives muteI2CFor.
 * ---------------------------------------------------------------------- */

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
    /* Vierte und damit LETZTE freie Adresse: I2C_BB_MAX_ADDRESSES ist 4. Die
     * uebrigen Aufrufe ignorieren ihren Rueckgabewert, ein fuenfter Eintrag
     * wuerde also still scheitern -- hier deshalb laut. */
    if (!i2c_bb_slave_register(ALMEMO_MENU_I2C_ADDR, onWrite, onRead))
        LOG_ERROR("I2CSlave: Menue-Adresse 0x%02x nicht registriert, Tabelle voll (max %u)",
                  (unsigned)ALMEMO_MENU_I2C_ADDR, (unsigned)I2C_BB_MAX_ADDRESSES);
    NVIC_EnableIRQ(GPIOTE_IRQn);
    muteUntilMs = 0;
    LOG_INFO("I2CSlave: un-muted, addresses re-registered");
}

/* -------------------------------------------------------------------------
 * onValue — mesh -> slot table update (called from Router task)
 * ---------------------------------------------------------------------- */

void I2CSlaveThread::onValue(uint32_t node_id, uint8_t sub, const char unit[2], int8_t exponent, int16_t raw)
{
    /* Only temperatures go into the emulated EEPROM; everything else (humidity,
     * pressure, …) is received but dropped here. */
    if (!isTemperatureUnit(unit)) {
        LOG_DEBUG("AlmemoRx: skip non-temp value node=%08X sub=%u unit=%02X%02X", (unsigned)node_id, sub,
                  (uint8_t)unit[0], (uint8_t)unit[1]);
        return;
    }

    uint8_t slot = findOrAllocSlot(node_id, sub);
    if (slot == 0xFF)
        return; /* all 4 channels taken — ignore this new stream */

    SlotEntry &n = slots[slot];
    if (!n.in_use) {
        /* First value for this (node_id, sub): claim the slot, fill its EEPROM
         * info block and ask the I2CSlave task to mute the bus so the master
         * rescans and discovers the new sensor. We only set the flag here (single
         * atomic write); runOnce() (I2CSlave task) owns the actual mute. The two
         * tasks are NOT synchronized, so runOnce() may start the mute between
         * initSlotForNode() and the writeRawForSlot() below — that is harmless:
         * muting only unregisters the I2C addresses, it never touches reg_data, and
         * the value write still lands before the master can read again (after unmute). */
        n.in_use  = true;
        n.node_id = node_id;
        n.sub     = sub;
        initSlotForNode(slot, node_id, sub, unit, exponent);
        rescanRequested = true;
        LOG_INFO("AlmemoRx: slot %u allocated for node=%08X sub=%u (rescan requested)", slot, (unsigned)node_id, sub);
    }

    /* Update (or first-write) the live value; the slot keeps it until the next
     * value arrives — no expiration. */
    slots[slot].raw      = raw;
    slots[slot].exponent = exponent;
    writeRawForSlot(slot, raw);
}

/* -------------------------------------------------------------------------
 * OSThread runOnce — mute state machine + stats log + stuck-state debug
 *
 * No slot expiration: slots are filled first-come and keep their last value
 * forever. Only this (I2CSlave) task touches the mute, so no cross-task race.
 * ---------------------------------------------------------------------- */

int32_t I2CSlaveThread::runOnce()
{
    /* Mute state machine — only this (I2CSlave) task touches muteUntilMs and the
     * i2c_bb_slave registration, so there is no cross-task race. onValue() (Router
     * task) merely sets rescanRequested. While muted we just wait for the window
     * to end; otherwise a pending rescan request starts a fresh mute. */
    if (muteUntilMs) {
        if ((int32_t)(millis() - muteUntilMs) >= 0)
            unmuteI2C(); // window over → audible again
    } else if (rescanRequested) {
        rescanRequested = false;

        /* Slot-Tabelle hat sich geaendert -> Menuestruktur neu bauen: die
         * Messwert-Labels kommen aus slots[], und der neue Hash sagt dem Node,
         * dass er die Struktur neu holen muss.
         *
         * BEWUSST OHNE NVIC_DisableIRQ: die ISR liefert Bloecke direkt aus menu
         * aus, ein Rebuild mittendrin schickt also eine zerrissene Struktur
         * raus. Das faengt der Hash ab -- der Node prueft die geholte Struktur
         * dagegen und holt bei Abweichung erneut, und dann ist sie stabil.
         * IRQs zu sperren waere das groessere Risiko: der Rebuild dauert ~1 ms,
         * und in der Zeit wuerden wir auch den Logger nicht hoeren, der 0x50
         * regelmaessig zur Steckerkennung pollt. */
        buildMenu();

        /* Only mute within the master's startup window; later the master won't
         * re-enumerate anyway, so a mute would just disrupt the bus for nothing. */
        if (millis() < ALMEMO_RX_MUTE_WINDOW_MS)
            muteI2CFor(ALMEMO_RX_MUTE_MS); // new sensor pending → start a rescan mute
        else
            LOG_DEBUG("AlmemoRx: rescan skipped, past %lums startup window", (unsigned long)ALMEMO_RX_MUTE_WINDOW_MS);
    }

    /* Edit aus dem Menue-Kanal abarbeiten (die ISR hat ihn nur abgelegt). Die
     * Bedeutung steht im eigenen Menue -- der Host schickt nur den Index. */
    if (s_edit_pending) {
        NVIC_DisableIRQ(GPIOTE_IRQn);
        const uint8_t it  = s_edit_item;
        const int32_t raw = s_edit_raw;
        s_edit_pending    = false;
        NVIC_EnableIRQ(GPIOTE_IRQn);

        if (it < menu.itemCount) {
            if (applyMenuSetting(menu.items[it].setting, raw))
                buildMenu(); /* Preset-Text aendert sich -> neuer Hash, Node holt neu */
        } else {
            LOG_WARN("I2CSlave: Edit auf Item %u ausserhalb des Menues (%u Items)", (unsigned)it,
                     (unsigned)menu.itemCount);
        }
    }

    /* Laufende Menuewerte einsammeln. Billig (kein Bus-Zugriff) und nur aus
     * diesem Task -- die ISR liest menuVals, kann dabei aber hoechstens einen
     * halb aktualisierten Werteblock ausliefern. Das ist unkritisch: Werte sind
     * ohnehin Momentaufnahmen, und der naechste Abruf korrigiert es. Fuer die
     * Struktur gilt das nicht, dort haengt die Konsistenz am Hash. */
    pollMenuValues();

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
        /* Invariant: jede committete Write-Transaktion hat mindestens ihr erstes
         * Datenbyte quittiert, also acks >= writes. Weniger ACKs als
         * Transaktionen waere eine echte Anomalie.
         *
         * NICHT geprueft wird Gleichheit. writes zaehlt TRANSAKTIONEN,
         * acks_sent zaehlt BYTES (so dokumentiert in i2c_bb_slave.h) -- gleich
         * sind sie nur, solange jeder Write genau ein Byte traegt. Das galt fuer
         * die alten 1-Byte-Adresszeiger und ist mit dem 16-Bit-Zeiger des
         * Menue-Kanals hinfaellig: dort kommen 2 Byte pro Transaktion, und
         * acks waechst entsprechend doppelt so schnell.
         *
         * Ebenso nicht geprueft: wr == rd. Der Master macht Writes ohne
         * folgenden Read (Adress-Probes zur Steckerkennung). */
        if (acks < writes) {
            LOG_WARN("I2CSlave ack deficit: wr=%lu rd=%lu ack=%lu", writes, reads, acks);
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

        /* Selbstheilung. Der In-ISR-Timeout kann hier nichts machen: er braucht
         * einen GPIOTE-Event, und bei irq_delta == 0 kommen genau die nicht mehr.
         * Beobachtet mit unserem Node als Master -- dessen TWIM stoesst bei einem
         * Write ohne Datenbyte (MAXCNT=0) den STOP ohne LASTTX-Warten an, der
         * landet also sehr dicht hinter dem ACK und eine Flanke geht verloren.
         * Am ALMEMO-Logger tritt der Fall nicht auf. */
        i2c_bb_slave_reset();
        LOG_INFO("I2CSlave: reset nach Haenger (resets=%lu)", (unsigned long)i2c_bb_slave_stats.resets);
        stuckSinceMs    = 0; /* Erkennung neu starten */
        stuckIrqAtStart = 0;
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
    } else if (addr == ALMEMO_MENU_I2C_ADDR) {
        /* 16-Bit-Adresszeiger, big endian (hi zuerst) -- die Struktur ist rund
         * 1,5 kB, ein Byte wuerde nicht reichen. Edits (itemIdx + raw) kommen
         * spaeter als laengerer Write dazu; bis dahin alles andere verwerfen. */
        if (len == 2) {
            s_menu_ptr = (uint16_t)((buf[0] << 8) | buf[1]);
        } else if (len == ALMEMO_MENU_EDIT_LEN &&
                   (uint16_t)((buf[0] << 8) | buf[1]) == ALMEMO_MENU_OFF_EDIT) {
            /* Nur merken -- angewandt wird im Task, s. runOnce(). */
            s_edit_item    = buf[2];
            s_edit_raw     = (int32_t)((uint32_t)buf[3] | ((uint32_t)buf[4] << 8) |
                                       ((uint32_t)buf[5] << 16) | ((uint32_t)buf[6] << 24));
            s_edit_pending = true;
        }
    }
}

/* -------------------------------------------------------------------------
 * buildMenu — eigene Menuestruktur fuer den Menue-Kanal
 *
 * Aufbau wie beim Node, mit den Abweichungen aus dem Menuekonzept:
 *   MESSWERTE      je belegtem Slot eine Zeile, Label wie im Logger-Kommentar
 *                  ("NNNN.M<c>.<v>"), KEIN Countdown -- wir senden nicht
 *   FUNK           ChUtil, AirUtilTX, Verbundene Nodes; kein Paketzaehler und
 *                  kein "Gesendet vor"
 *   SYSTEM         Typ, Kennung, Version, Firmware (+ Dirty, s. APP_DIRTY);
 *                  es gibt keine Sensor-Seite
 *   EINSTELLUNGEN  identisch zum Node (geteilte Listen, s. AlmemoMenuSettings.h)
 *
 * Reihenfolge ist wichtig, zweimal:
 *   - ALLE Pool-Texte muessen vor finish() eingetragen sein, sonst deckt der
 *     Hash sie nicht ab.
 *   - Die Menueseite muss vor der Statusseite stehen, die sie als target
 *     referenziert.
 * ---------------------------------------------------------------------- */

/* Wert-Indizes des Receiver-Menues. Die Messwerte liegen vorn, danach die
 * festen Felder -- so wie beim Node, damit beide Seiten gleich zu lesen sind. */
enum : uint8_t {
    VAL_MEAS0 = 0,
    VAL_CHUTIL = VAL_MEAS0 + I2CSlaveThread::MAX_SLOTS,
    VAL_AIRUTIL,
    VAL_NODES,
    VAL_TYPE,
    VAL_IDENT,
    VAL_VER,
    VAL_FIRMWARE,
#if APP_DIRTY
    VAL_DIRTY,
#endif
    VAL_PRESET,
    VAL_DISPLAY,        /* aktuelles Display; gefuellt vom Host */
    VAL_DISPLAY_TARGET, /* Ziel des Toggle-Items; gefuellt vom Host */
    VAL_MENU_COUNT,
};

void I2CSlaveThread::buildMenu()
{
    MenuBuilder b(menu);

    /* Kennung und Version aus dem EEPROM, damit Node und Logger dasselbe
     * sehen; ALMEMO fuellt die Felder mit Leerzeichen auf. */
    char ident[9], ver[9];
    memcpy(ident, &eeprom[0][0x00], 8);
    memcpy(ver, &eeprom[0][0xF8], 8);
    ident[8] = ver[8] = '\0';
    for (int i = 7; i >= 0 && ident[i] == ' '; i--)
        ident[i] = '\0';
    const char *verTrim = ver;
    while (*verTrim == ' ')
        verTrim++;

    /* Kopfzeile links, wie beim Node die vier Hex-Stellen der Node-Nummer --
     * hier mit Praefix, damit man dem Display ansieht, wessen Menue es zeigt.
     * KURZ halten: header() zeichnet links bei x=0 und den Seitentitel
     * rechtsbuendig bei x=127, ohne Kollisionsschutz. "GW" + 4 Stellen braucht
     * ~42 px, der breiteste Titel ("MESSWERTE 1/2") ~78 px -- das passt auf
     * 128 px. Mit "LoRa-GW" davor waere es ~66 px und wuerde ueberlappen. */
    char head[16];
    snprintf(head, sizeof(head), "GW %04X", (unsigned)(nodeDB ? (nodeDB->getNodeNum() & 0xFFFF) : 0));
    b.deviceName(head);

    /* Texte vorab in den Pool -- nach finish() darf nichts mehr dazu. */
    offIdent    = b.str(ident[0] ? ident : "-");
    offVer      = b.str(verTrim[0] ? verTrim : "-");
    offFirmware = b.str(optstr(APP_VERSION));
#if APP_DIRTY
    offDirty    = b.str("yes");
#endif
    /* "Typ" ist die ALMEMO-Generation, die wir emulieren -- aus der Version
     * abgeleitet, genau wie der Node es tut (6.xx = D6, 7.xx = D7). */
    offType = b.str(verTrim[0] == '7' ? "D7" : "D6");

    /* Messwert-Labels: Node-Kurzname + Messstelle, dasselbe Format wie im
     * Kommentarfeld des Loggers (s. initSlotForNode) -- "NNNN.M<c>.<v>" bzw.
     * "NNNN.<v>" bei Kanal 0. So zeigen Display und Logger dieselbe Bezeichnung. */
    char lbl[MAX_SLOTS][14];
    uint8_t used[MAX_SLOTS];
    uint8_t n = 0;
    for (uint8_t i = 0; i < MAX_SLOTS; i++) {
        if (!slots[i].in_use)
            continue;
        const unsigned node16  = (unsigned)(slots[i].node_id & 0xFFFF);
        const uint8_t  channel = almemoSlotChannel(slots[i].sub);
        const uint8_t  valIdx  = almemoSlotValueIndex(slots[i].sub);
        if (channel == 0)
            snprintf(lbl[n], sizeof(lbl[0]), "%04X.%u", node16, valIdx);
        else
            snprintf(lbl[n], sizeof(lbl[0]), "%04X.M%u.%u", node16, channel - 1, valIdx);
        used[n++] = i;
    }

    /* --- Auswahllisten: geteilt mit dem Node, s. AlmemoMenuSettings.h.
     *     OHNE Intervall: der Receiver sendet keine ALMEMO-Pakete, ein
     *     Sendeintervall hat hier keine Bedeutung. --- */
    const uint8_t enumPreset  = menudef::presetEnum(b, offPreset);
    const uint8_t enumDisplay = menudef::displayEnum(b);

    /* --- Messwerte. Kein Countdown wie beim Node: der Receiver sendet nicht,
     *     er empfaengt -- es gibt also keine "naechste Aktualisierung". --- */
    const uint8_t pages = n ? (uint8_t)((n + MENU_ROWS - 1) / MENU_ROWS) : 1;
    char title[20];
    for (uint8_t pg = 0; pg < pages; pg++) {
        if (pages > 1)
            snprintf(title, sizeof(title), "MESSWERTE %u/%u", (unsigned)(pg + 1), (unsigned)pages);
        else
            snprintf(title, sizeof(title), "MESSWERTE");
        b.page(title, MenuPageKind::Status);
        if (!n) {
            b.item("Keine Messstellen", MENU_NO_IDX, nullptr, MenuItemKind::Hint);
            continue;
        }
        for (uint8_t r = 0; r < MENU_ROWS; r++) {
            const uint8_t i = (uint8_t)(pg * MENU_ROWS + r);
            if (i >= n)
                break;
            b.item(lbl[i], (uint8_t)(VAL_MEAS0 + used[i]));
        }
    }

    /* --- Funk. Kein "Pakete" und kein "Gesendet vor" -- der Receiver sendet
     *     nicht; dafuer die Zahl der Nodes, von denen Pakete kamen. --- */
    b.page("FUNK", MenuPageKind::Status);
    b.item("ChUtil", VAL_CHUTIL, "%");
    b.item("AirUtilTX", VAL_AIRUTIL, "%");
    b.item("Verbundene Nodes", VAL_NODES);

    /* --- System: beim Receiver umfangreicher, es gibt keine Sensor-Seite. --- */
    b.page("SYSTEM", MenuPageKind::Status);
    b.item("Typ", VAL_TYPE);
    b.item("Kennung", VAL_IDENT);
    b.item("Version", VAL_VER);
    b.item("Firmware", VAL_FIRMWARE);
#if APP_DIRTY
    b.item("Dirty", VAL_DIRTY);
#endif

    /* --- Einstellungen: identisch zum Node. Die Menueseite MUSS vor der
     *     Statusseite stehen, die sie als target referenziert. --- */
    /* Titel gekuerzt: links steht "GW XXXX", und mit "EINSTELLUNGEN" rechts
     * ueberschneiden sich beide in der Mitte -- header() zeichnet ohne
     * Kollisionsschutz. */
    const uint8_t pageMenu = menu.pageCount;
    b.page("EINSTELL.", MenuPageKind::Menu);
    b.item("Radio-Mode", VAL_PRESET, nullptr, MenuItemKind::EditEnum, enumPreset, MENU_NO_IDX,
           config.lora.use_preset ? 0 : MENU_ITEM_DISABLED, MENU_SETTING_PRESET);
    /* Toggle: ein Druck schaltet um, ohne Auswahlliste und Bestaetigung.
     * Angezeigt wird das ZIEL; den aktuellen Zustand zeigt die Status-Seite. */
    b.item("Display ->", VAL_DISPLAY_TARGET, nullptr, MenuItemKind::Toggle, enumDisplay, MENU_NO_IDX, 0,
           MENU_SETTING_DISPLAY);
    b.item("<- Zurück", MENU_NO_IDX, nullptr, MenuItemKind::Back);

    b.page("EINSTELL.", MenuPageKind::Status, MENU_PAGE_OPENS_MENU, pageMenu);
    b.item("Radio-Mode", VAL_PRESET);
    b.item("Display", VAL_DISPLAY, nullptr, MenuItemKind::Display, enumDisplay, MENU_NO_IDX, 0,
           MENU_SETTING_DISPLAY);
    b.item("", MENU_NO_IDX, nullptr, MenuItemKind::Hint);
    b.item("Lang drücken: ändern", MENU_NO_IDX, nullptr, MenuItemKind::Hint);

    b.finish(); /* ab hier NICHTS mehr in den Pool legen -- der Hash ist gebildet */

    /* Die Werte selbst gehen noch nicht ueber den Bus; der Werteblock
     * (ALMEMO_MENU_OFF_VALUES) ist der naechste Schritt. Die Offsets der festen
     * Texte stehen aber schon fest und liegen im Pool. */
    s_menu_hdr.magic        = ALMEMO_MENU_MAGIC;
    s_menu_hdr.modelVersion = ALMEMO_MENU_MODEL_VERSION;
    s_menu_hdr.pageCount    = menu.pageCount;
    s_menu_hdr.itemCount    = menu.itemCount;
    s_menu_hdr.valueCount   = menu.valueCount;
    s_menu_hdr.enumCount    = menu.enumCount;
    s_menu_hdr.optionCount  = menu.optionCount;
    s_menu_hdr.numberCount  = menu.numberCount;
    s_menu_hdr.poolLen      = menu.poolLen;
    s_menu_hdr.structSize   = (uint16_t)sizeof(MenuStruct);
    s_menu_hdr.hash         = menu.hash;

    LOG_INFO("I2CSlave: Menue gebaut (%u Seiten, %u Items, %u Werte, %u Pool, %u B, hash %08lx)", menu.pageCount,
             menu.itemCount, menu.valueCount, menu.poolLen, (unsigned)sizeof(MenuStruct),
             (unsigned long)menu.hash);
}

/* -------------------------------------------------------------------------
 * applyMenuSetting — eine Einstellung nach ihrer Bedeutung anwenden
 *
 * Gegenstueck zu AlmemoMenuLocal::applySetting(). Identifiziert wird ueber
 * MENU_SETTING_*, nicht ueber den Item-Index -- der gilt nur im eigenen Menue.
 * ---------------------------------------------------------------------- */

bool I2CSlaveThread::applyMenuSetting(uint8_t setting, int32_t raw)
{
    switch (setting) {
    case MENU_SETTING_PRESET:
        config.lora.modem_preset = (meshtastic_Config_LoRaConfig_ModemPreset)raw;
        config.lora.use_preset   = true;
        /* reloadConfig() statt saveToDisk(): es macht DREI Dinge, und das
         * Speichern ist nur das letzte davon --
         *   nodeDB->resetRadioConfig();           Region/Frequenzslot neu rechnen
         *   configChanged.notifyObservers(NULL);  Funkmodul uebernimmt die Parameter
         *   nodeDB->saveToDisk(saveWhat);
         * Hier stand nur der letzte Schritt, die Einstellung war also
         * gespeichert, aber das Funkmodul erfuhr nie davon und sendete bis
         * zum naechsten Einschalten mit den alten Parametern weiter. Von
         * aussen sieht das aus wie "man muss neu starten" -- ein Neustart
         * ist aber NICHT noetig: AdminModule setzt fuer LoRa-Aenderungen
         * ausdruecklich `requiresReboot = false` und verlaesst sich auf
         * denselben Observer (s. AdminModule.cpp:838). reconfigure() legt
         * das Radio in Standby, programmiert alle Modem-Parameter neu und
         * startet den Empfang. */
        service->reloadConfig(SEGMENT_CONFIG);
        LOG_INFO("I2CSlave: Preset -> %s",
                 DisplayFormatters::getModemPresetDisplayName(config.lora.modem_preset, false, true));
        return true;
    default:
        LOG_WARN("I2CSlave: Einstellung %u nicht unterstuetzt", (unsigned)setting);
        return false;
    }
}

/* -------------------------------------------------------------------------
 * pollMenuValues — laufende Werte fuer den Werteblock
 *
 * Gegenstueck zu AlmemoMenuLocal::poll() auf der Node-Seite, mit denselben
 * Quellen (airTime, config) -- damit beide Geraete dieselben Zahlen zeigen.
 * Laeuft aus runOnce(), nicht aus der ISR.
 * ---------------------------------------------------------------------- */

bool I2CSlaveThread::pollMenuValues()
{
    MenuValue before[MENU_MAX_VALUES];
    memcpy(before, menuVals, sizeof(before));

    auto setFixed = [this](uint8_t idx, int32_t raw, int8_t exp) {
        menuVals[idx].kind     = (uint8_t)MenuValueKind::Fixed;
        menuVals[idx].raw      = raw;
        menuVals[idx].exponent = exp;
    };
    auto setText = [this](uint8_t idx, uint16_t off) {
        menuVals[idx].kind = (uint8_t)MenuValueKind::Text;
        menuVals[idx].text = off;
    };

    /* Messwerte je Slot. Ein nie befuellter Slot zeigt "----" (Invalid) statt 0. */
    uint8_t  nodeCount = 0;
    uint32_t seen[MAX_SLOTS];
    for (uint8_t i = 0; i < MAX_SLOTS; i++) {
        if (!slots[i].in_use) {
            menuVals[VAL_MEAS0 + i].kind = (uint8_t)MenuValueKind::Invalid;
            continue;
        }
        setFixed((uint8_t)(VAL_MEAS0 + i), slots[i].raw, slots[i].exponent);

        /* "Verbundene Nodes" zaehlt Absender, nicht Messstellen: mehrere Slots
         * desselben Nodes zaehlen einmal. */
        bool known = false;
        for (uint8_t k = 0; k < nodeCount; k++)
            if (seen[k] == slots[i].node_id) {
                known = true;
                break;
            }
        if (!known)
            seen[nodeCount++] = slots[i].node_id;
    }

    menuVals[VAL_NODES].kind = (uint8_t)MenuValueKind::Int;
    menuVals[VAL_NODES].raw  = nodeCount;

    if (airTime) {
        setFixed(VAL_CHUTIL, (int32_t)(airTime->channelUtilizationPercent() * 10.0f), -1);
        setFixed(VAL_AIRUTIL, (int32_t)(airTime->utilizationTXPercent() * 10.0f), -1);
    }

    /* Feste Texte: Offsets stehen seit buildMenu() im Pool. */
    setText(VAL_TYPE, offType);
    setText(VAL_IDENT, offIdent);
    setText(VAL_VER, offVer);
    setText(VAL_FIRMWARE, offFirmware);
#if APP_DIRTY
    setText(VAL_DIRTY, offDirty);
#endif

    /* Einstellungen, aus derselben Config wie beim Node. */
    {
        const uint8_t p = (uint8_t)config.lora.modem_preset;
        menuVals[VAL_PRESET].kind = (uint8_t)MenuValueKind::Text;
        menuVals[VAL_PRESET].text = (p < 16 && offPreset[p]) ? offPreset[p] : MENU_NO_TEXT;
        menuVals[VAL_PRESET].raw  = (int32_t)config.lora.modem_preset;
    }

    return memcmp(before, menuVals, sizeof(before)) != 0;
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
    } else if (addr == ALMEMO_MENU_I2C_ADDR) {
        /* Flaches Fenster, s. AlmemoMenuWire.h: Kopf bei 0x0000, Struktur bei
         * 0x0020. Der Werteblock (0x0800) kommt noch nicht -- Lesen dort
         * liefert 0, der Master sieht NACK und weiss, dass nichts da ist. */
        const uint8_t *src = nullptr;
        uint16_t       rem = 0;

        if (s_menu_ptr < ALMEMO_MENU_OFF_HEADER + sizeof(s_menu_hdr)) {
            const uint16_t off = (uint16_t)(s_menu_ptr - ALMEMO_MENU_OFF_HEADER);
            src                = (const uint8_t *)&s_menu_hdr + off;
            rem                = (uint16_t)(sizeof(s_menu_hdr) - off);
        } else if (s_menu_ptr >= ALMEMO_MENU_OFF_STRUCT &&
                   s_menu_ptr < ALMEMO_MENU_OFF_STRUCT + sizeof(MenuStruct)) {
            const uint16_t off = (uint16_t)(s_menu_ptr - ALMEMO_MENU_OFF_STRUCT);
            src                = (const uint8_t *)&s_instance->menu + off;
            rem                = (uint16_t)(sizeof(MenuStruct) - off);
        } else if (s_menu_ptr >= ALMEMO_MENU_OFF_VALUES &&
                   s_menu_ptr < ALMEMO_MENU_OFF_VALUES + sizeof(s_instance->menuVals)) {
            const uint16_t off = (uint16_t)(s_menu_ptr - ALMEMO_MENU_OFF_VALUES);
            src                = (const uint8_t *)s_instance->menuVals + off;
            rem                = (uint16_t)(sizeof(s_instance->menuVals) - off);
        } else {
            return 0;
        }

        uint8_t len = rem > max_len ? max_len : (uint8_t)rem;
        for (uint8_t i = 0; i < len; i++)
            buf[i] = src[i];
        s_menu_ptr = (uint16_t)(s_menu_ptr + len);
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
