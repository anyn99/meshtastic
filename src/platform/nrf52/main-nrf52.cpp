#include "configuration.h"
#include <Adafruit_TinyUSB.h>
#include <Adafruit_nRFCrypto.h>
#include <InternalFileSystem.h>
#include <SPI.h>
#include <Wire.h>

#define APP_WATCHDOG_SECS 90
#define NRFX_WDT_ENABLED 1
#define NRFX_WDT0_ENABLED 1
#define NRFX_WDT_CONFIG_NO_IRQ 1
#include "nrfx_power.h"
#include <assert.h>
#include <ble_gap.h>
#include <memory.h>
// nrfx_qspi.h's NRFX_QSPI_DEFAULT_CONFIG macro references this priority symbol,
// which the Adafruit BSP only declares in nrfx_config.h when Adafruit_SPIFlash
// is linked — we aren't, so define it here before the include. Value 7 = lowest.
#define NRFX_QSPI_DEFAULT_CONFIG_IRQ_PRIORITY 7
#include <nrfx_qspi.h>
#include <nrfx_wdt.c>
#include <nrfx_wdt.h>
#include <stdio.h>
// #include <Adafruit_USBD_Device.h>
#include "NodeDB.h"
#include "PowerMon.h"
#include "error.h"
#include "main.h"
#include "meshUtils.h"
#include "power.h"
#include <power/PowerHAL.h>

#include <hal/nrf_lpcomp.h>

#ifdef BQ25703A_ADDR
#include "BQ25713.h"
#endif

// WARNING! THRESHOLD + HYSTERESIS should be less than regulated VDD voltage - which depends on board
// and is 3.0 or 3.3V. Also VDD likes to read values like 2.9999 so make sure you account for that
// otherwise board will not boot at all. Before you modify this part - please triple read NRF52840 power design
// section in datasheet and you understand how REG0 and REG1 regulators work together.
#ifndef SAFE_VDD_VOLTAGE_THRESHOLD
#define SAFE_VDD_VOLTAGE_THRESHOLD 2.7
#endif

// hysteresis value
#ifndef SAFE_VDD_VOLTAGE_THRESHOLD_HYST
#define SAFE_VDD_VOLTAGE_THRESHOLD_HYST 0.2
#endif

uint16_t getVDDVoltage();

// Weak empty variant shutdown prep function.
// May be redefined by variant files.
void variant_shutdown() __attribute__((weak));
void variant_shutdown() {}

static nrfx_wdt_t nrfx_wdt = NRFX_WDT_INSTANCE(0);
static nrfx_wdt_channel_id nrfx_wdt_channel_id_nrf52_main;

// This is a public global so that the debugger can set it to false automatically from our gdbinit
// @phaseloop comment: most part of codebase, including filesystem flash driver depend on softdevice
// methods so disabling it may actually crash thing. Proceed with caution.

bool useSoftDevice = true; // Set to false for easier debugging

static inline void debugger_break(void)
{
    __asm volatile("bkpt #0x01\n\t"
                   "mov pc, lr\n\t");
}

// PowerHAL NRF52 specific function implementations
bool powerHAL_isVBUSConnected()
{
    return NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk;
}

bool powerHAL_isPowerLevelSafe()
{

    static bool powerLevelSafe = true;

    uint16_t threshold = SAFE_VDD_VOLTAGE_THRESHOLD * 1000; // convert V to mV
    uint16_t hysteresis = SAFE_VDD_VOLTAGE_THRESHOLD_HYST * 1000;

    if (powerLevelSafe) {
        if (getVDDVoltage() < threshold) {
            powerLevelSafe = false;
        }
    } else {
        // power level is only safe again when it raises above threshold + hysteresis
        if (getVDDVoltage() >= (threshold + hysteresis)) {
            powerLevelSafe = true;
        }
    }

    return powerLevelSafe;
}

void powerHAL_platformInit()
{

    // Enable POF power failure comparator. It will prevent writing to NVMC flash when supply voltage is too low.
    // Set to some low value as last resort - powerHAL_isPowerLevelSafe uses different method and should manage proper node
    // behaviour on its own.

    // POFWARN is pretty useless for node power management because it triggers only once and clearing this event will not
    // re-trigger it again until voltage rises to safe level and drops again. So we will use SAADC routed to VDD to read safely
    // voltage.

    // @phaseloop: I disable POFCON for now because it seems to be unreliable or buggy. Even when set at 2.0V it
    // triggers below 2.8V and corrupts data when pairing bluetooth - because it prevents filesystem writes and
    // adafruit BLE library triggers lfs_assert which reboots node and formats filesystem.
    // I did experiments with bench power supply and no matter what is set to POFCON, it always triggers right below
    // 2.8V. I compared raw registry values with datasheet.

    NRF_POWER->POFCON =
        ((POWER_POFCON_THRESHOLD_V22 << POWER_POFCON_THRESHOLD_Pos) | (POWER_POFCON_POF_Enabled << POWER_POFCON_POF_Pos));

    // remember to always match VBAT_AR_INTERNAL with AREF_VALUE in variant definition file
#ifdef VBAT_AR_INTERNAL
    analogReference(VBAT_AR_INTERNAL);
#else
    analogReference(AR_INTERNAL); // 3.6V
#endif
}

// get VDD voltage (in millivolts)
uint16_t getVDDVoltage()
{
    // we use the same values as regular battery read so there is no conflict on SAADC
    analogReadResolution(BATTERY_SENSE_RESOLUTION_BITS);

    // VDD range on NRF52840 is 1.8-3.3V so we need to remap analog reference to 3.6V
    // let's hope battery reading runs in same task and we don't have race condition
    analogReference(AR_INTERNAL);

    uint16_t vddADCRead = analogReadVDD();
    float voltage = ((1000 * 3.6) / pow(2, BATTERY_SENSE_RESOLUTION_BITS)) * vddADCRead;

// restore default battery reading reference
#ifdef VBAT_AR_INTERNAL
    analogReference(VBAT_AR_INTERNAL);
#endif

    return voltage;
}

bool loopCanSleep()
{
    // turn off sleep only while connected via USB
    // return true;
    return !Serial; // the bool operator on the nrf52 serial class returns true if connected to a PC currently
    // return !(TinyUSBDevice.mounted() && !TinyUSBDevice.suspended());
}

// handle standard gcc assert failures
void __attribute__((noreturn)) __assert_func(const char *file, int line, const char *func, const char *failedexpr)
{
    LOG_ERROR("assert failed %s: %d, %s, test=%s", file, line, func, failedexpr);
    // debugger_break(); FIXME doesn't work, possibly not for segger
    // Reboot cpu
    NVIC_SystemReset();
}

void getMacAddr(uint8_t *dmac)
{
    const uint8_t *src = (const uint8_t *)NRF_FICR->DEVICEADDR;
    dmac[5] = src[0];
    dmac[4] = src[1];
    dmac[3] = src[2];
    dmac[2] = src[3];
    dmac[1] = src[4];
    dmac[0] = src[5] | 0xc0; // MSB high two bits get set elsewhere in the bluetooth stack
}

#if !MESHTASTIC_EXCLUDE_BLUETOOTH
void setBluetoothEnable(bool enable)
{
    // For debugging use: don't use bluetooth
    if (!useSoftDevice) {
        if (enable)
            LOG_INFO("Disable NRF52 BLUETOOTH WHILE DEBUGGING");
        return;
    }

    // If user disabled bluetooth: init then disable advertising & reduce power
    // Workaround. Avoid issue where device hangs several days after boot..
    // Allegedly, no significant increase in power consumption
    if (!config.bluetooth.enabled) {
        static bool initialized = false;
        if (!initialized) {
            nrf52Bluetooth = new NRF52Bluetooth();
            nrf52Bluetooth->startDisabled();
            initialized = true;
        }
        return;
    }

    if (enable) {
        powerMon->setState(meshtastic_PowerMon_State_BT_On);

        // If not yet set-up
        if (!nrf52Bluetooth) {
            LOG_DEBUG("Init NRF52 Bluetooth");
            nrf52Bluetooth = new NRF52Bluetooth();
            nrf52Bluetooth->setup();
        }
        // Already setup, apparently
        else
            nrf52Bluetooth->resumeAdvertising();
    }
    // Disable (if previously set-up)
    else if (nrf52Bluetooth) {
        powerMon->clearState(meshtastic_PowerMon_State_BT_On);
        nrf52Bluetooth->shutdown();
    }
}
#else
#warning NRF52 "Bluetooth disable" workaround does not apply to builds with MESHTASTIC_EXCLUDE_BLUETOOTH
void setBluetoothEnable(bool enable) {}
#endif
/**
 * Override printf to use the SEGGER output library (note - this does not effect the printf method on the debug console)
 */
int printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    auto res = SEGGER_RTT_vprintf(0, fmt, &args);
    va_end(args);
    return res;
}

namespace
{
constexpr uint8_t NRF52_MAGIC_LFS_IS_CORRUPT = 0xF5;
constexpr uint32_t MULTIPLE_CORRUPTION_DELAY_MILLIS = 20 * 60 * 1000;
static unsigned long millis_until_formatting_again = 0;

// Report the critical error from loop(), giving a chance for the screen to be initialized first.
inline void reportLittleFSCorruptionOnce()
{
    static bool report_corruption = !!millis_until_formatting_again;
    if (report_corruption) {
        report_corruption = false;
        RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_FLASH_CORRUPTION_UNRECOVERABLE);
    }
}
} // namespace

void preFSBegin()
{
    // The GPREGRET register keeps its value across warm boots. Check that this is a warm boot and, if GPREGRET
    // is set to NRF52_MAGIC_LFS_IS_CORRUPT, format LittleFS.
    if (!(NRF_POWER->RESETREAS == 0 && NRF_POWER->GPREGRET == NRF52_MAGIC_LFS_IS_CORRUPT))
        return;
    NRF_POWER->GPREGRET = 0;
    millis_until_formatting_again = millis() + MULTIPLE_CORRUPTION_DELAY_MILLIS;
    InternalFS.format();
    LOG_INFO("LittleFS format complete; restoring default settings");
}

extern "C" void lfs_assert(const char *reason)
{
    LOG_ERROR("LittleFS corruption detected: %s", reason);
    if (millis_until_formatting_again > millis()) {
        RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_FLASH_CORRUPTION_UNRECOVERABLE);
        const long millis_remain = millis_until_formatting_again - millis();
        LOG_WARN("Pausing %d seconds to avoid wear on flash storage", millis_remain / 1000);
        delay(millis_remain);
    }
    LOG_INFO("Rebooting to format LittleFS");
    delay(500); // Give the serial port a bit of time to output that last message.
    // Try setting GPREGRET with the SoftDevice first. If that fails (perhaps because the SD hasn't been initialize yet) then set
    // NRF_POWER->GPREGRET directly.

    // TODO: this will/can crash CPU if bluetooth stack is not compiled in or bluetooth is not initialized
    // (regardless if enabled or disabled) - as there is no live SoftDevice stack
    // implement "safe" functions detecting softdevice stack state and using proper method to set registers

    // do not set GPREGRET if POFWARN is triggered because it means lfs_assert reports flash undervoltage protection
    // and not data corruption. Reboot is fine as boot procedure will wait until power level is safe again

    if (!NRF_POWER->EVENTS_POFWARN) {
        if (!(sd_power_gpregret_clr(0, 0xFF) == NRF_SUCCESS &&
              sd_power_gpregret_set(0, NRF52_MAGIC_LFS_IS_CORRUPT) == NRF_SUCCESS)) {
            NRF_POWER->GPREGRET = NRF52_MAGIC_LFS_IS_CORRUPT;
        }
    }

    // TODO: this should not be done when SoftDevice is enabled as device will not boot back on soft reset
    // as some data is retained in RAM which will prevent re-enabling bluetooth stack
    // Google what Nordic has to say about NVIC_* + SoftDevice
    NVIC_SystemReset();
}

void checkSDEvents()
{
    if (useSoftDevice) {
        uint32_t evt;
        while (NRF_SUCCESS == sd_evt_get(&evt)) {
            switch (evt) {
            case NRF_EVT_POWER_FAILURE_WARNING:
                RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_BROWNOUT);
                break;

            default:
                LOG_DEBUG("Unexpected SDevt %d", evt);
                break;
            }
        }
    } else {
        if (NRF_POWER->EVENTS_POFWARN)
            RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_BROWNOUT);
    }
}

void nrf52Loop()
{
    {
        static bool watchdog_running = false;
        if (!watchdog_running) {
            nrfx_wdt_enable(&nrfx_wdt);
            watchdog_running = true;
        }
    }
    nrfx_wdt_channel_feed(&nrfx_wdt, nrfx_wdt_channel_id_nrf52_main);

    checkSDEvents();
    reportLittleFSCorruptionOnce();
}

#ifdef USE_SEMIHOSTING
#include <SemihostingStream.h>
#include <meshUtils.h>

/**
 * Note: this variable is in BSS and therfore false by default.  But the gdbinit
 * file will be installing a temporary breakpoint that changes wantSemihost to true.
 */
bool wantSemihost;

/**
 * Turn on semihosting if the ICE debugger wants it.
 */
void nrf52InitSemiHosting()
{
    if (wantSemihost) {
        static SemihostingStream semiStream;
        // We must dynamically alloc because the constructor does semihost operations which
        // would crash any load not talking to a debugger
        semiStream.open();
        semiStream.println("Semihosting starts!");
        // Redirect our serial output to instead go via the ICE port
        console->setDestination(&semiStream);
    }
}
#endif

#ifdef HFCLK_DBG_PIN
// Mirror HFCLKSTAT.STATE on a GPIO so we can see HFXO state during sleep with
// PPK2/scope. D0 on XIAO = P0.02 (g_ADigitalPinMap[0] = 2).
//   D0 = HIGH while HFCLKSTAT.STATE = 1 (HFXO running, ~700 µA penalty)
//   D0 = LOW  while HFCLKSTAT.STATE = 0 (HFINT only, cheap)
//
// nRF52 has EVENTS_HFCLKSTARTED but NO matching HFCLKSTOPPED event, so we can't
// fully track HFXO state in pure hardware (PPI+GPIOTE) — only the rising edge.
// Instead we replace the FreeRTOS port's WFE polling loop (port_cmsis_systick.c
// :211 — `do { __WFE(); } while (0 == NVIC->ISPR)`) with our own that re-mirrors
// HFCLKSTAT after every __WFE() return. That includes phantom event-bus wakes
// (peripheral EVENTs without an enabled IRQ — invisible to the FreeRTOS loop),
// so D0 follows HFXO at the granularity of those wake events. configPRE_SLEEP_
// PROCESSING signals "skip your loop" to FreeRTOS by setting the modifiable
// idle time to 0.
//
// We also accumulate per-IRQ wake counts in a .noinit array that survives
// NVIC_SystemReset, so the totals build up across sender cycles. The dump is
// printed at boot via dumpPowerDiag; plugging USB to read it adds at most one
// USB-loaded cycle's IRQs on top of many no-USB cycles.
#define HFCLK_DBG_IRQ_COUNT 48
#define HFCLK_DBG_IRQ_MAGIC 0x6E43514Au // 'IQCn' v2 — bump to force re-init after layout change (added sleep_* mirrors)
static volatile uint32_t hfclk_irq_counts[HFCLK_DBG_IRQ_COUNT] __attribute__((section(".noinit")));
static volatile uint32_t hfclk_phantom_wakes __attribute__((section(".noinit")));
static volatile uint32_t hfclk_total_wakes __attribute__((section(".noinit")));
static volatile uint32_t hfclk_irq_magic __attribute__((section(".noinit")));
// Sleep-only mirrors of the above. Updated only while hfclk_sleep_active=true,
// which cpuDeepSleep flips on right before delay(msecToWake). Lets us separate
// boot/TX wakes from the actual sleep-window wakes.
static volatile uint32_t sleep_irq_counts[HFCLK_DBG_IRQ_COUNT] __attribute__((section(".noinit")));
static volatile uint32_t sleep_phantom_wakes __attribute__((section(".noinit")));
static volatile uint32_t sleep_total_wakes __attribute__((section(".noinit")));
// In .bss (auto-zeroed each boot) — sleep window doesn't survive across reset.
static volatile bool hfclk_sleep_active = false;

extern "C" {
void hfclk_dbg_init(void)
{
    NRF_P0->DIRSET = (1u << HFCLK_DBG_PIN);
    NRF_P0->OUTCLR = (1u << HFCLK_DBG_PIN);
}
void hfclk_dbg_irq_init(void)
{
    // Cold-boot only: any non-magic value (uninitialised RAM after power-on, or
    // we changed the layout) means the array is invalid and must be zeroed.
    // After NVIC_SystemReset RAM is preserved, so the magic stays valid and the
    // counters keep accumulating.
    if (hfclk_irq_magic != HFCLK_DBG_IRQ_MAGIC) {
        for (uint32_t i = 0; i < HFCLK_DBG_IRQ_COUNT; i++) {
            hfclk_irq_counts[i] = 0;
            sleep_irq_counts[i] = 0;
        }
        hfclk_phantom_wakes = 0;
        hfclk_total_wakes = 0;
        sleep_phantom_wakes = 0;
        sleep_total_wakes = 0;
        hfclk_irq_magic = HFCLK_DBG_IRQ_MAGIC;
    }
}
void hfclk_dbg_sleep_mark(int active) { hfclk_sleep_active = (active != 0); }
static const char *const hfclk_irq_names[HFCLK_DBG_IRQ_COUNT] = {
    "POWER_CLOCK", "RADIO",   "UARTE0",   "TWIM0_TWIS0", "SPIM1_TWIM1", "NFCT",     "GPIOTE",  "SAADC",
    "TIMER0",      "TIMER1",  "TIMER2",   "RTC0",        "TEMP",        "RNG",      "ECB",     "CCM_AAR",
    "WDT",         "RTC1",    "QDEC",     "COMP_LPCOMP", "SWI0_EGU0",   "SWI1_EGU1","SWI2_EGU2","SWI3_EGU3",
    "SWI4_EGU4",   "SWI5_EGU5","TIMER3",  "TIMER4",      "PWM0",        "PDM",      "RES30",   "RES31",
    "MWU",         "PWM1",    "PWM2",     "SPIM2_SPIS2", "RTC2",        "I2S",      "FPU",     "USBD",
    "UARTE1",      "QSPI",    "CRYPTOCELL","RES43",      "RES44",       "PWM3",     "RES46",   "SPIM3"};
void hfclk_dbg_irq_dump(void)
{
    LOG_INFO("WakeIRQ: total=%u phantom=%u  sleep-only: total=%u phantom=%u (magic=0x%08x)",
             (unsigned)hfclk_total_wakes, (unsigned)hfclk_phantom_wakes,
             (unsigned)sleep_total_wakes, (unsigned)sleep_phantom_wakes, (unsigned)hfclk_irq_magic);
    for (uint32_t i = 0; i < HFCLK_DBG_IRQ_COUNT; i++) {
        if (hfclk_irq_counts[i] || sleep_irq_counts[i])
            LOG_INFO("  IRQ%2u %-12s total=%u sleep=%u", (unsigned)i, hfclk_irq_names[i],
                     (unsigned)hfclk_irq_counts[i], (unsigned)sleep_irq_counts[i]);
    }
}
static inline void hfclk_dbg_mirror(void)
{
    if (NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_STATE_Msk)
        NRF_P0->OUTSET = (1u << HFCLK_DBG_PIN);
    else
        NRF_P0->OUTCLR = (1u << HFCLK_DBG_PIN);
}
// Drop-in replacement for the FreeRTOS WFE polling loop. Pre-conditions match
// port_cmsis_systick.c: PRIMASK is set, so WFE wakes from event-bus signals OR
// pending-IRQ bits but no IRQ is dispatched. We sample HFCLKSTAT after every
// wake and exit only when an NVIC IRQ has actually pended. Per-IRQ + phantom
// counters are updated each iteration.
void hfclk_dbg_pre_sleep(uint32_t *pIdleTime)
{
    // Clear sticky FPU exception flags (IDC|IXC|UFC|OFC|DZC|IOC) and pending
    // FPU_IRQn — lazy stacking would otherwise re-pend it inside our WFE loop.
    __set_FPSCR(__get_FPSCR() & ~0x9Fu);
    (void)__get_FPSCR();
    NVIC_ClearPendingIRQ(FPU_IRQn);
    // USBD pending bit can stick around even after USBD->ENABLE=0; clear once.
    NVIC_ClearPendingIRQ(USBD_IRQn);

    hfclk_dbg_mirror();

    while (1) {
        __WFE();
        hfclk_dbg_mirror();
        uint32_t i0 = NVIC->ISPR[0];
        uint32_t i1 = NVIC->ISPR[1];
        bool s = hfclk_sleep_active;
        hfclk_total_wakes++;
        if (s) sleep_total_wakes++;
        if ((i0 | i1) == 0) {
            hfclk_phantom_wakes++;
            if (s) sleep_phantom_wakes++;
            continue;
        }
        for (uint32_t b = 0; b < 32; b++)
            if (i0 & (1u << b)) {
                hfclk_irq_counts[b]++;
                if (s) sleep_irq_counts[b]++;
            }
        for (uint32_t b = 0; b < 16; b++)
            if (i1 & (1u << b)) {
                hfclk_irq_counts[32 + b]++;
                if (s) sleep_irq_counts[32 + b]++;
            }
        break;
    }

    // Tell FreeRTOS to skip its own `do{__WFE();}while(...)` — we already slept.
    *pIdleTime = 0;
}
// Re-mirror once more for the post-IRQ state, then OR the pending-IRQ bitmap
// into GPREGRET2 (survives NVIC_SystemReset for boot-time decoding). Layout:
//   bit 7 = validity marker
//   bit 6 = any-other (anything not in the named slots below)
//   bit 5 = USBD        (IRQ 39, ISPR[1] bit 7)
//   bit 4 = RTC1        (IRQ 17, ISPR[0] bit 17)
//   bit 3 = RTC0        (IRQ 11, ISPR[0] bit 11)
//   bit 2 = TIMER0      (IRQ  8, ISPR[0] bit 8)
//   bit 1 = GPIOTE      (IRQ  6, ISPR[0] bit 6)
//   bit 0 = POWER_CLOCK (IRQ  0, ISPR[0] bit 0)
void hfclk_dbg_post_sleep(uint32_t /*idleTicks*/)
{
    hfclk_dbg_mirror();

    uint32_t i0 = NVIC->ISPR[0];
    uint32_t i1 = NVIC->ISPR[1];
    uint8_t add = 0x80;
    if (i0 & (1u << 0)) add |= 0x01;
    if (i0 & (1u << 6)) add |= 0x02;
    if (i0 & (1u << 8)) add |= 0x04;
    if (i0 & (1u << 11)) add |= 0x08;
    if (i0 & (1u << 17)) add |= 0x10;
    if (i1 & (1u << 7)) add |= 0x20;
    uint32_t named0 = (1u << 0) | (1u << 6) | (1u << 8) | (1u << 11) | (1u << 17);
    uint32_t named1 = (1u << 7);
    if ((i0 & ~named0) || (i1 & ~named1)) add |= 0x40;
    NRF_POWER->GPREGRET2 = (uint8_t)(NRF_POWER->GPREGRET2 | add);
}
}
#endif

#ifdef SEEED_XIAO_NRF52840_KIT
// XIAO BLE Sense carries a P25Q16H QSPI flash that this build doesn't use
// (EXTERNAL_FLASH_USE_QSPI is commented out in variant.h, Adafruit_SPIFlash isn't
// linked). Out of reset the chip sits in standby (~5–20 µA). Sending command
// 0xB9 (Deep Power Down) drops it to ~1 µA, where it stays as long as VCC is
// applied — across NVIC_SystemReset, only a hardware power-cycle wakes it.
//
// Uses Nordic's nrfx_qspi driver directly (it's compiled into the BSP via
// NRFX_QSPI_ENABLED=1 for nRF52840). This is the same code path Adafruit's
// Adafruit_FlashTransport_QSPI.begin()/runCommand(0xB9)/end() uses internally.
// Pin map per variant.cpp: SCK=P0.21 CSN=P0.25 IO0=P0.20 IO1=P0.24 IO2=P0.22 IO3=P0.23
static void xiaoQspiFlashDpd()
{
    //LOG_INFO("XIAO QSPI flash: pre HFCLKSTAT=0x%08x ENABLE=%u",
    //         (unsigned)NRF_CLOCK->HFCLKSTAT, (unsigned)NRF_QSPI->ENABLE);

    // Some Adafruit BSP boot paths leave QSPI in a half-initialised state from
    // a previous run / framework startup hook. Force a hardware power-cycle of
    // the peripheral via the hidden POWER register (peripheral_base + 0xFFC),
    // a documented Nordic workaround for "peripheral stuck" cases.
    //*(volatile uint32_t *)(NRF_QSPI_BASE + 0xFFC) = 0;
    //__NOP(); __NOP(); __NOP();
    //*(volatile uint32_t *)(NRF_QSPI_BASE + 0xFFC) = 1;

    // QSPI activation needs HFCLK from HFXO. With SoftDevice off and BLE/radio
    // not yet up at this point in boot, only the 16 MHz HFINT runs — QSPI never
    // becomes ready and nrfx_qspi_init returns NRFX_ERROR_TIMEOUT (0xBAD0007).
    // Kick HFXO manually, balance with HFCLKSTOP after.
	/*
    bool hfxoOwned = false;
    if (!(NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_STATE_Msk)) {
        NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
        NRF_CLOCK->TASKS_HFCLKSTART = 1;
        for (uint32_t to = 200000; NRF_CLOCK->EVENTS_HFCLKSTARTED == 0 && to; --to) {}
        hfxoOwned = true;
    }
    LOG_INFO("XIAO QSPI flash: post-HFXO HFCLKSTAT=0x%08x (owned=%d)",
             (unsigned)NRF_CLOCK->HFCLKSTAT, (int)hfxoOwned);
	*/
    // Use the QSPI peripheral's built-in DPM (Deep Power-down Mode) support
    // instead of fighting with raw cinstr. With dpmconfig=true and dpmen=true
    // *before* ACTIVATE, the peripheral knows the flash is currently in DPM
    // and skips any flash probing — ACTIVATE comes up cleanly even when the
    // last cycle's 0xB9 left the chip asleep (state persists across reset).
    // After init we transition DPMEN 1→0 → peripheral auto-sends 0xAB (release)
    // and waits DPMDUR (exit) before any further bus activity.
    // DPMDUR units are 256×16 MHz cycles = 16 µs per count. P25Q16H tRDP ≤ 30 µs,
    // 0x10 (256 µs) is a safe over-spec; entry side gets the same.
    NRF_QSPI->DPMDUR = (0x0010 << 16) | 0x0010;
    nrfx_qspi_config_t cfg = NRFX_QSPI_DEFAULT_CONFIG(21, 25, 20, 24, 22, 23);
    cfg.prot_if.dpmconfig = true;
    cfg.phy_if.dpmen = true; // assume flash IS in DPM until proven otherwise
    nrfx_err_t err = nrfx_qspi_init(&cfg, NULL, NULL);
    if (err == NRFX_SUCCESS) {
        // 1→0 transition: peripheral generates Release-from-DPD (0xAB) and waits
        // DPMDUR.EXIT before returning to ready.
        NRF_QSPI->IFCONFIG1 &= ~QSPI_IFCONFIG1_DPMEN_Msk;
        for (uint32_t to = 5000; to && !NRF_QSPI->EVENTS_READY; --to)
            delayMicroseconds(10);
        NRF_QSPI->EVENTS_READY = 0;
        // Flash is awake (or was awake all along). Send our own 0xB9 via cinstr
        // to (re-)enter DPD. We could equally use the DEACTIVATE path with
        // DPMEN=1, but explicit cinstr is clearer and avoids state surprises.
        nrfx_err_t cerr = nrfx_qspi_cinstr_quick_send(0xB9, NRF_QSPI_CINSTR_LEN_1B, NULL);
        LOG_INFO("XIAO QSPI flash: DPD %s", (cerr == NRFX_SUCCESS) ? "sent (cmd 0xB9)" : "send failed");
    } else {
        // Init still failed. Force-disable so a half-initialised peripheral
        // doesn't keep clock requests alive (was the source of the 5 mA floor).
        LOG_WARN("XIAO QSPI flash: nrfx_qspi_init failed (err=0x%x) HFCLKSTAT=0x%08x", (unsigned)err,
                 (unsigned)NRF_CLOCK->HFCLKSTAT);


    }
    nrfx_qspi_uninit();
    NRF_QSPI->TASKS_DEACTIVATE = 1;
    NRF_QSPI->ENABLE = 0;
    *(volatile uint32_t *)(NRF_QSPI_BASE + 0xFFC) = 0;
    __NOP(); __NOP(); __NOP();
    *(volatile uint32_t *)(NRF_QSPI_BASE + 0xFFC) = 1;
    NRF_QSPI->ENABLE = 0;

    //if (hfxoOwned)
    //    NRF_CLOCK->TASKS_HFCLKSTOP = 1;

    // After uninit the pins fall back to GPIO with no drive. Pin every QSPI line
    // to a defined level so floating pads don't leak via the input buffer.
    // SCK=P0.21, CSN=P0.25, IO0=P0.20, IO1=P0.24, IO2=P0.22, IO3=P0.23.
    // CSN HIGH so flash stays deselected. SCK + IO0 + IO1 LOW (idle/don't-care).
    // IO2 (WP#) HIGH — write-protect inactive. IO3 (HOLD#/RESET# on Puya) HIGH —
    // driven LOW would assert reset on some flashes.
    auto driveOut = [](uint32_t pin, bool high) {
        NRF_P0->PIN_CNF[pin] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
        if (high)
            NRF_P0->OUTSET = (1u << pin);
        else
            NRF_P0->OUTCLR = (1u << pin);
    };
    driveOut(21, false); // SCK
    driveOut(20, false); // IO0
    driveOut(24, false); // IO1
    driveOut(22, true);  // IO2 / WP#
    driveOut(23, true);  // IO3 / HOLD#
    driveOut(25, true);  // CSN
}
#endif

void nrf52Setup()
{
#ifdef ADC_V
    pinMode(ADC_V, INPUT);
#endif

    uint32_t why = NRF_POWER->RESETREAS;
    // per
    // https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.nrf52832.ps.v1.1%2Fpower.html
    LOG_DEBUG("Reset reason: 0x%x", why);

    // Decode accumulated pending-IRQ bitmap from PRE/POST_SLEEP hooks during the
    // last delay() in cpuDeepSleep. GPREGRET2 survives NVIC_SystemReset; bit 7 is
    // our validity marker (bootloader clears RESETREAS so we can't trust SREQ).
    {
        uint8_t v = (uint8_t)NRF_POWER->GPREGRET2;
        if (v & 0x80) {
            LOG_INFO("Pre-reset wake-IRQs: POWER_CLOCK=%u GPIOTE=%u TIMER0=%u RTC0=%u "
                     "RTC1=%u USBD=%u other=%u (raw=0x%02x)",
                     !!(v & 0x01), !!(v & 0x02), !!(v & 0x04), !!(v & 0x08),
                     !!(v & 0x10), !!(v & 0x20), !!(v & 0x40), v);
            NRF_POWER->GPREGRET2 = 0;
        }
    }

#ifdef USE_SEMIHOSTING
    nrf52InitSemiHosting();
#endif

    // Per
    // https://devzone.nordicsemi.com/nordic/nordic-blog/b/blog/posts/monitor-mode-debugging-with-j-link-and-gdbeclipse
    // This is the recommended setting for Monitor Mode Debugging
    NVIC_SetPriority(DebugMonitor_IRQn, 6UL);

#ifdef BQ25703A_ADDR
    auto *bq = new BQ25713();
    if (!bq->setup())
        LOG_ERROR("ERROR! Charge controller init failed");
#endif

#ifdef SEEED_XIAO_NRF52840_KIT
    xiaoQspiFlashDpd();
#endif

    // Init random seed
    union seedParts {
        uint32_t seed32;
        uint8_t seed8[4];
    } seed;
    nRFCrypto.begin();
    nRFCrypto.Random.generate(seed.seed8, sizeof(seed.seed8));
    LOG_DEBUG("Set random seed %u", seed.seed32);
    randomSeed(seed.seed32);
    nRFCrypto.end();

    // Set up nrfx watchdog. Do not enable the watchdog yet (we do that
    // the first time through the main loop), so that other threads can
    // allocate their own wdt channel to protect themselves from hangs.
    nrfx_wdt_config_t wdt0_config = {
        .behaviour = NRF_WDT_BEHAVIOUR_PAUSE_SLEEP_HALT, .reload_value = APP_WATCHDOG_SECS * 1000,
        // Note: Not using wdt interrupts.
        // .interrupt_priority = NRFX_WDT_DEFAULT_CONFIG_IRQ_PRIORITY
    };
    nrfx_err_t r = nrfx_wdt_init(&nrfx_wdt, &wdt0_config,
                                 nullptr // Watchdog event handler, not used, we just reset.
    );
    assert(r == NRFX_SUCCESS);

    r = nrfx_wdt_channel_alloc(&nrfx_wdt, &nrfx_wdt_channel_id_nrf52_main);
    assert(r == NRFX_SUCCESS);

#ifdef NRF52_USE_DCDC
    // Switch the chip's INTERNAL VDD→1.3V regulator (REG1) from LDO to DCDC.
    // Requires the L1 inductor between DCC and VDD on the PCB — opt in per variant.
    // useSoftDevice is a compile-time default; check the actual runtime state,
    // since SD-less builds (BLE excluded) leave SVC calls as silent no-ops (err=2).
    {
        uint8_t sdEn = 0;
        sd_softdevice_is_enabled(&sdEn);
        if (sdEn) {
            uint32_t err = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
            LOG_INFO("Enable DCDC (REG1) via SD: err=%u", err);
        } else {
            NRF_POWER->DCDCEN = 1;
            LOG_INFO("Enable DCDC (REG1) via direct register (SD off)");
        }
    }
#endif

#ifdef HFCLK_DBG_PIN
    hfclk_dbg_init();
    hfclk_dbg_irq_init();
#endif
}

void cpuDeepSleep(uint32_t msecToWake)
{

    // FIXME, configure RTC or button press to wake us
    // FIXME, power down SPI, I2C, RAMs
#if HAS_WIRE
    Wire.end();
#endif
    SPI.end();
#if SPI_INTERFACES_COUNT > 1
    SPI1.end();
#endif


    if (Serial)       // Another check in case of disabled default serial, does nothing bad
        Serial.end(); // This may cause crashes as debug messages continue to flow.

        // This causes troubles with waking up on nrf52 (on pro-micro in particular):
        // we have no Serial1 in use on nrf52, check Serial and GPS modules.
#ifdef PIN_SERIAL1_RX
    if (Serial1) // A straightforward solution to the wake from deepsleep problem
        Serial1.end();
#endif

    setBluetoothEnable(false);

#ifdef RAK4630
#ifdef PIN_3V3_EN
    digitalWrite(PIN_3V3_EN, LOW);
#endif
#ifdef AQ_SET_PIN
    // RAK-12039 set pin for Air quality sensor
    digitalWrite(AQ_SET_PIN, LOW);
#endif
#endif
    // Run shutdown code if specified in variant.cpp
    variant_shutdown();

    // Sleepy trackers or sensors can low power "sleep"
    // Don't enter this if we're sleeping portMAX_DELAY, since that's a shutdown event
    if (msecToWake != portMAX_DELAY &&
        (IS_ONE_OF(config.device.role, meshtastic_Config_DeviceConfig_Role_TRACKER,
                   meshtastic_Config_DeviceConfig_Role_TAK_TRACKER, meshtastic_Config_DeviceConfig_Role_SENSOR) &&
         config.power.is_power_saving == true)) {
        // sd_power_mode_set is SD-managed; with SD off the SVC silently fails
        // (err=2) and direct POWER register writes would HardFault under SD.
        // Without SD, trigger TASKS_LOWPWR directly — LOWPWR is the reset
        // default but some EasyDMA peripherals can implicitly switch to
        // CONSTLAT, so set it explicitly.
        uint8_t sdEn = 0;
        sd_softdevice_is_enabled(&sdEn);
        if (sdEn)
            sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
        else
            NRF_POWER->TASKS_LOWPWR = 1;

        // Pin SX126x NSS HIGH so the chip stays in its commanded sleep state. After
        // SPI.end() the pin can float and a transient low pulse drops the chip back
        // to standby_RC (~600 µA). Same trick the ESP32 branch in sleep.cpp uses.
#ifdef SX126X_CS
        pinMode(SX126X_CS, OUTPUT);
        digitalWrite(SX126X_CS, HIGH);
#endif
        // Drive RXEN LOW so the antenna switch / RX frontend bias path is off.
#ifdef SX126X_RXEN
        pinMode(SX126X_RXEN, OUTPUT);
        digitalWrite(SX126X_RXEN, LOW);
#endif

        // After SPI.end() the SCK/MOSI/MISO pins fall back to default GPIO (input,
        // buffer connected, no pull). With the SX126x asleep nothing drives MISO,
        // so it floats and the input buffer leaks. Drive SCK + MOSI LOW (output)
        // and disconnect MISO's input buffer entirely (no buffer = no leakage).
#ifdef PIN_SPI_SCK
        {
            auto driveLow = [](uint32_t arduinoPin) {
                NRF_GPIO_Type *port = digitalPinToPort(arduinoPin);
                uint32_t mask = digitalPinToBitMask(arduinoPin);
                uint32_t idx = __builtin_ctz(mask);
                port->PIN_CNF[idx] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
                port->OUTCLR = mask;
            };
            auto disconnectInput = [](uint32_t arduinoPin) {
                NRF_GPIO_Type *port = digitalPinToPort(arduinoPin);
                uint32_t mask = digitalPinToBitMask(arduinoPin);
                uint32_t idx = __builtin_ctz(mask);
                port->PIN_CNF[idx] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                     (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
            };
            driveLow(PIN_SPI_SCK);
            driveLow(PIN_SPI_MOSI);
            disconnectInput(PIN_SPI_MISO);
        }
#endif

        // Detach USB pull-up and disable the USBD peripheral so its clocks/regulator
        // stop during the sleep window. Safe because we NVIC_SystemReset() below — the
        // BSP re-inits USB on next boot.
        NRF_USBD->USBPULLUP = 0;
        NRF_USBD->ENABLE = 0;
        NVIC_DisableIRQ(USBD_IRQn);
        NVIC_ClearPendingIRQ(USBD_IRQn);

        // Silence CRYPTOCELL: even with NVIC IRQ disabled, SEVONPEND wakes WFE on
        // every IRQ-line pulse (saw ~54 wakes/cycle in the wake-IRQ counter). The
        // peripheral was only briefly used by nRFCrypto.begin() at boot for the seed.
        NRF_CRYPTOCELL->ENABLE = 0;
        NVIC_ClearPendingIRQ(CRYPTOCELL_IRQn);

#ifdef SEEED_XIAO_NRF52840_KIT
        // XIAO BLE Sense board carries an LSM6DS3 IMU (6D_PWR=P1.08) and PDM mic
        // (MIC_PWR=P1.10) whose supply rails are gated by these pins. The BSP
        // never initialises them, so they float — drive them LOW so the IMU and
        // mic can't pull any current during the sleep window.
        NRF_P1->PIN_CNF[8] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
        NRF_P1->OUTCLR = (1u << 8);
        NRF_P1->PIN_CNF[10] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
        NRF_P1->OUTCLR = (1u << 10);
        // PDM mic data/clock lines (CLK=P1.00, DATA=P0.16) terminate at the now-
        // unpowered MSM261D3526H1CPM. CLK is normally driven by the nRF but PDM
        // is uninitialised here, so it floats. DATA is high-Z from the dead mic.
        // Drive both LOW.
        NRF_P1->PIN_CNF[0] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
        NRF_P1->OUTCLR = (1u << 0);
        NRF_P0->PIN_CNF[16] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
        NRF_P0->OUTCLR = (1u << 16);
        // The IMU's I2C lines (SCL=P0.27, SDA=P0.07) carry 10k pull-ups to the
        // IMU's VCC rail. With 6D_PWR LOW that rail is dead and the pull-ups end
        // up against a floating supply — the lines settle at some intermediate
        // voltage and the nRF input buffers leak. INT1 (P0.11) is an output from
        // the unpowered IMU, also high-Z. Drive all three LOW.
        NRF_P0->PIN_CNF[27] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
        NRF_P0->OUTCLR = (1u << 27);
        NRF_P0->PIN_CNF[7] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
        NRF_P0->OUTCLR = (1u << 7);
        NRF_P0->PIN_CNF[11] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
        NRF_P0->OUTCLR = (1u << 11);
#endif

        // Battery sense divider (BATTERY_PIN tap + ADC_CTRL sink). After the last
        // battery_adcDisable() the sink sits OUTPUT HIGH (~3.3V) — divider still
        // leaks ~260 nA against VBAT. Worse, BATTERY_PIN is left as digital INPUT
        // with the buffer connected; the ~VBAT/3 tap voltage sits in the buffer's
        // linear region and can draw single-digit µA. Disconnect both input
        // buffers — the divider node floats up to VBAT, no current anywhere.
#if defined(BATTERY_PIN) && defined(ADC_CTRL)
        {
            auto disconnectInput = [](uint32_t arduinoPin) {
                NRF_GPIO_Type *port = digitalPinToPort(arduinoPin);
                uint32_t mask = digitalPinToBitMask(arduinoPin);
                uint32_t idx = __builtin_ctz(mask);
                port->PIN_CNF[idx] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                     (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
            };
            disconnectInput(BATTERY_PIN);
            disconnectInput(ADC_CTRL);
        }
#endif

        // BQ25101 charger pins. HICHG (ISET select) is OUTPUT LOW from variant init
        // — sinking the BQ's internal pull-up costs ~1 µA. EXT_CHRG_DETECT (~CHG)
        // is an open-drain status output with no external pull-up on the XIAO; the
        // input buffer leaks while the line floats. Disconnect both — HICHG floats
        // up via the BQ pull-up (selects 50 mA charge, irrelevant in sleep), and
        // variant.cpp re-asserts LOW on next boot.
#if defined(HICHG) || defined(EXT_CHRG_DETECT)
        {
            auto disconnectInput = [](uint32_t arduinoPin) {
                NRF_GPIO_Type *port = digitalPinToPort(arduinoPin);
                uint32_t mask = digitalPinToBitMask(arduinoPin);
                uint32_t idx = __builtin_ctz(mask);
                port->PIN_CNF[idx] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                     (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
            };
#ifdef HICHG
            disconnectInput(HICHG);
#endif
#ifdef EXT_CHRG_DETECT
            disconnectInput(EXT_CHRG_DETECT);
#endif
        }
#endif

        // Cycling USBD won't drop HFXO if nrfx_clock still holds an explicit request.
        // Force-stop HFCLK so we drop to the 16MHz internal RC (or fully off in WFI).
        NRF_CLOCK->TASKS_HFCLKSTOP = 1;
#if 0
        // nRF52 errata workaround: TWIM/SPIM/UARTE/USBD can leave HFCLK + EasyDMA
        // running even after ENABLE=0 if a transaction wasn't explicitly stopped first.
        // Each peripheral has a hidden POWER register at offset 0xFFC; writing 0→1
        // forces a full power-cycle and releases all clock/DMA requests.
        // Ref: https://devzone.nordicsemi.com/ "UARTE current consumption"
        auto cyclePower = [](uintptr_t base) {
            *(volatile uint32_t *)(base + 0xFFC) = 0;
            (void)*(volatile uint32_t *)(base + 0xFFC);
            *(volatile uint32_t *)(base + 0xFFC) = 1;
        };
        cyclePower(0x40003000); // TWIM0 / SPIM0 / SPIS0 / TWIS0 (shared)
        cyclePower(0x40004000); // TWIM1 / SPIM1 / SPIS1 / TWIS1 (shared)
        cyclePower(0x40023000); // SPIM2 / SPIS2
        cyclePower(0x40002000); // UARTE0
        cyclePower(0x40028000); // UARTE1
        cyclePower(0x40027000); // USBD


        // Real deep sleep: bypass FreeRTOS entirely. vTaskDelay keeps the scheduler
        // ticking and every queued wakeup pulls the chip out of WFI — that was the
        // 800 µA baseline + 7 ms spike pattern. After switching to RTC2+WFE we still
        // saw 1 ms spikes: with SEVONPEND any *pending* IRQ wakes WFE even with
        // PRIMASK=1, and FreeRTOS's RTC1 tick was still firing every ~977 µs. So we
        // must physically stop every other wake source first.
        __disable_irq();

        // Kill the FreeRTOS tick (RTC1) and all its event/interrupt sources.
        NRF_RTC1->TASKS_STOP = 1;
        NRF_RTC1->INTENCLR = 0xFFFFFFFF;
        NRF_RTC1->EVTENCLR = 0xFFFFFFFF;
        // Belt-and-suspenders: also stop RTC0 in case anything else owns it.
        NRF_RTC0->TASKS_STOP = 1;
        NRF_RTC0->INTENCLR = 0xFFFFFFFF;
        NRF_RTC0->EVTENCLR = 0xFFFFFFFF;

        // Mask every NVIC IRQ source and clear pending bits, so SEVONPEND won't
        // wake us on stray events from peripherals we haven't explicitly stopped.
        for (uint32_t i = 0; i < 8; i++) {
            NVIC->ICER[i] = 0xFFFFFFFF;
            NVIC->ICPR[i] = 0xFFFFFFFF;
        }

        SCB->SCR |= SCB_SCR_SEVONPEND_Msk | SCB_SCR_SLEEPDEEP_Msk;

        // RTC2 runs from LFCLK (32768 Hz, ~0.5 µA), 24-bit counter — for an 8-9 s
        // sleep window PRESCALER=0 is fine (max ~512 s).
        NRF_RTC2->TASKS_STOP = 1;
        NRF_RTC2->TASKS_CLEAR = 1;
        NRF_RTC2->PRESCALER = 0;
        uint64_t ticks = ((uint64_t)msecToWake * 32768ULL) / 1000ULL;
        if (ticks < 2) ticks = 2;            // RTC errata: CC must be ≥ COUNTER+2
        if (ticks > 0x00FFFFFE) ticks = 0x00FFFFFE;
        NRF_RTC2->CC[0] = (uint32_t)ticks;
        NRF_RTC2->EVENTS_COMPARE[0] = 0;
        NRF_RTC2->INTENSET = RTC_INTENSET_COMPARE0_Msk;
        NVIC_ClearPendingIRQ(RTC2_IRQn);
        // Deliberately do NOT NVIC_EnableIRQ — we only want the event to wake WFE,
        // not to dispatch into an ISR. SEVONPEND makes this work with PRIMASK set.

        NRF_RTC2->TASKS_START = 1;

        while (NRF_RTC2->EVENTS_COMPARE[0] == 0) {
            __SEV();
            __WFE(); // clears event flag set by SEV
            __WFE(); // actually sleeps until COMPARE0 pends RTC2_IRQn
        }

        NRF_RTC2->TASKS_STOP = 1;
#endif

        // Reset hook counter to 0 (with validity bit set). The PRE_SLEEP hook
        // increments it on every invocation during the upcoming delay(). On the
        // next boot the count reveals how often tickless idle was aborted —
        // i.e. how often something woke us during the supposed deep sleep.
        NRF_POWER->GPREGRET2 = 0x80;

#ifdef HFCLK_DBG_PIN
        extern void hfclk_dbg_sleep_mark(int);
        hfclk_dbg_sleep_mark(1);
#endif
        delay(msecToWake);
        NVIC_SystemReset();
    } else {
        // Resume on user button press
        // https://github.com/lyusupov/SoftRF/blob/81c519ca75693b696752235d559e881f2e0511ee/software/firmware/source/SoftRF/src/platform/nRF52.cpp#L1738
        constexpr uint32_t DFU_MAGIC_SKIP = 0x6d;
        sd_power_gpregret_clr(0, 0xFF);           // Clear the register before setting a new values in it for stability reasons
        sd_power_gpregret_set(0, DFU_MAGIC_SKIP); // Equivalent NRF_POWER->GPREGRET = DFU_MAGIC_SKIP

        // FIXME, use system off mode with ram retention for key state?
        // FIXME, use non-init RAM per
        // https://devzone.nordicsemi.com/f/nordic-q-a/48919/ram-retention-settings-with-softdevice-enabled

#ifdef BATTERY_LPCOMP_INPUT
        // Wake up if power rises again
        nrf_lpcomp_config_t c;
        c.reference = BATTERY_LPCOMP_THRESHOLD;
        c.detection = NRF_LPCOMP_DETECT_UP;
        c.hyst = NRF_LPCOMP_HYST_NOHYST;
        nrf_lpcomp_configure(NRF_LPCOMP, &c);
        nrf_lpcomp_input_select(NRF_LPCOMP, BATTERY_LPCOMP_INPUT);
        nrf_lpcomp_enable(NRF_LPCOMP);

        battery_adcEnable();

        nrf_lpcomp_task_trigger(NRF_LPCOMP, NRF_LPCOMP_TASK_START);
        while (!nrf_lpcomp_event_check(NRF_LPCOMP, NRF_LPCOMP_EVENT_READY))
            ;
#endif

        auto ok = sd_power_system_off();
        if (ok != NRF_SUCCESS) {
            LOG_ERROR("FIXME: Ignoring soft device (EasyDMA pending?) and forcing system-off!");
            NRF_POWER->SYSTEMOFF = 1;
        }
    }

    // The following code should not be run, because we are off
    while (1) {
        delay(5000);
        LOG_DEBUG(".");
    }
}

void clearBonds()
{
    if (!nrf52Bluetooth) {
        nrf52Bluetooth = new NRF52Bluetooth();
        nrf52Bluetooth->setup();
    }
    nrf52Bluetooth->clearBonds();
}

void enterDfuMode()
{
// SDK kit does not have native USB like almost all other NRF52 boards
#ifdef NRF_USE_SERIAL_DFU
    enterSerialDfu();
#else
    enterUf2Dfu();
#endif
}
