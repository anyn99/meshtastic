/**
 * @file  gpiote_dispatch.cpp
 * @brief GPIOTE IN-channel dispatch — replaces WInterrupts.c's half of the story.
 *
 * Problem
 * -------
 * I2CSlaveThread.cpp defines GPIOTE_IRQHandler as a strong C symbol, which
 * overrides the weak alias from startup.S *and* silently wins over
 * WInterrupts.c's equally strong definition (via -Wl,--allow-multiple-definition,
 * project .o files being first in the link order).
 *
 * WInterrupts.c never runs, so its private callbacksInt[] table — populated by
 * attachInterrupt() calls from Power.cpp (EXT_CHRG_DETECT) and RadioLib (DIO1) —
 * is never dispatched.  This means:
 *   • EVENTS_IN registers are never cleared → interrupt storm
 *   • RadioLib's isrTxLevel0 is never called → sendingPacket never freed → busyTx stuck
 *
 * Fix
 * ---
 * This file defines its own attachInterrupt / detachInterrupt that maintain a
 * callback table stored here, where gpiote_dispatch_events_in() can reach it.
 * I2CSlaveThread.cpp's GPIOTE_IRQHandler calls gpiote_dispatch_events_in() after
 * handling EVENTS_PORT (I2C), so both PORT events (I2C slave) and IN events
 * (radio DIO1, charge-detect, etc.) are handled correctly in one ISR.
 *
 * Because our symbols are project objects (linked before the framework archive)
 * they take precedence over WInterrupts.c's definitions of the same names.
 * WInterrupts.c's GPIOTE_IRQHandler, attachInterrupt, detachInterrupt are all
 * silently discarded by the linker.
 */

#include <nrf.h>
#include <string.h>

#include "Arduino.h"
#include "wiring_private.h"
#include "nrf_gpiote.h"

// ─── Helpers ──────────────────────────────────────────────────────────────────

#if defined(NRF52) || defined(NRF52_SERIES)
#define NUMBER_OF_GPIO_TE 8
#else
#define NUMBER_OF_GPIO_TE 4
#endif

#ifdef GPIOTE_CONFIG_PORT_Msk
#define GPIOTE_CONFIG_PORT_PIN_Msk (GPIOTE_CONFIG_PORT_Msk | GPIOTE_CONFIG_PSEL_Msk)
#else
#define GPIOTE_CONFIG_PORT_PIN_Msk GPIOTE_CONFIG_PSEL_Msk
#endif

// ─── Callback table ───────────────────────────────────────────────────────────

static voidFuncPtr s_callbacks[NUMBER_OF_GPIO_TE];
static int8_t      s_channelPin[NUMBER_OF_GPIO_TE]; /* nRF raw pin, or -1 = free */
static bool        s_initialized = false;

static void dispatch_init()
{
    if (s_initialized) return;
    s_initialized = true;
    memset(s_callbacks,  0,  sizeof(s_callbacks));
    memset(s_channelPin, -1, sizeof(s_channelPin));
    /*
     * NVIC priority and enable are handled by i2c_bb_slave_init(), which runs
     * after all attachInterrupt() calls.  We deliberately do NOT call
     * NVIC_SetPriority / NVIC_EnableIRQ here so that i2c_bb_slave_init() has
     * the final word on the priority (I2C_BB_IRQ_PRIORITY = 2).
     */
}

// ─── Public: dispatch called from GPIOTE_IRQHandler ──────────────────────────

/**
 * Process all pending GPIOTE IN events and invoke their registered callbacks.
 * Must be called from within GPIOTE_IRQHandler after EVENTS_PORT handling.
 * Clears each EVENTS_IN register before invoking the callback to prevent
 * re-entry (same pattern as WInterrupts.c).
 */
void gpiote_dispatch_events_in()
{
    /* Single volatile read covers all 8 channels. */
    uint32_t const intenset = NRF_GPIOTE->INTENSET;
    for (int ch = 0; ch < NUMBER_OF_GPIO_TE; ch++) {
        if (0 == (intenset & (1u << ch)))    continue;
        if (0 == NRF_GPIOTE->EVENTS_IN[ch]) continue;

        /* Clear before calling to prevent immediate re-storm if the callback
         * does not de-assert the pin.  Mirrors WInterrupts.c line 209. */
        NRF_GPIOTE->EVENTS_IN[ch] = 0;

        if (s_callbacks[ch]) {
            s_callbacks[ch]();
        }
    }
}

// ─── attachInterrupt / detachInterrupt ────────────────────────────────────────

extern "C" int attachInterrupt(uint32_t pin, voidFuncPtr callback, uint32_t mode)
{
    dispatch_init();

    if (pin >= PINS_COUNT) return 0;

    /* Map Arduino pin → nRF raw pin (same as WInterrupts.c line 75). */
    pin = g_ADigitalPinMap[pin];

    /* Strip ISR_DEFERRED flag (we don't support deferred callbacks). */
    mode &= ~(uint32_t)ISR_DEFERRED;

    uint32_t polarity;
    switch (mode) {
        case CHANGE:  polarity = GPIOTE_CONFIG_POLARITY_Toggle; break;
        case FALLING: polarity = GPIOTE_CONFIG_POLARITY_HiToLo; break;
        case RISING:  polarity = GPIOTE_CONFIG_POLARITY_LoToHi; break;
        default: return 0;
    }

    /* Pre-compute CONFIG register mask / bits (same logic as WInterrupts.c). */
    const uint32_t oldRegMask = ~(GPIOTE_CONFIG_PORT_PIN_Msk |
                                  GPIOTE_CONFIG_POLARITY_Msk  |
                                  GPIOTE_CONFIG_MODE_Msk      );
    const uint32_t newRegBits =
        ((pin      << GPIOTE_CONFIG_PSEL_Pos   ) & GPIOTE_CONFIG_PORT_PIN_Msk) |
        ((polarity << GPIOTE_CONFIG_POLARITY_Pos) & GPIOTE_CONFIG_POLARITY_Msk) |
        ((GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) & GPIOTE_CONFIG_MODE_Msk);

    /* Find existing channel for this pin, or a free one. */
    int  ch         = -1;
    bool newChannel = false;
    for (int i = 0; i < NUMBER_OF_GPIO_TE; i++) {
        if ((uint32_t)s_channelPin[i] == pin) { ch = i; break; }
    }
    if (ch == -1) {
        for (int i = 0; i < NUMBER_OF_GPIO_TE; i++) {
            if (s_channelPin[i] != -1) continue;
            if (nrf_gpiote_te_is_enabled(NRF_GPIOTE, i)) continue;
            ch = i; newChannel = true; break;
        }
    }
    if (ch == -1) return 0; /* all channels occupied */

    s_channelPin[ch] = (int8_t)pin;
    s_callbacks[ch]  = callback;

    uint32_t cfg = NRF_GPIOTE->CONFIG[ch];
    cfg &= oldRegMask;
    cfg |= newRegBits;
    NRF_GPIOTE->CONFIG[ch] = cfg;

    if (newChannel) {
        NRF_GPIOTE->EVENTS_IN[ch] = 0;
        NRF_GPIOTE->INTENSET = (1u << ch);
    }

    return (1 << ch);
}

extern "C" void detachInterrupt(uint32_t pin)
{
    if (pin >= PINS_COUNT) return;
    pin = g_ADigitalPinMap[pin];
    for (int ch = 0; ch < NUMBER_OF_GPIO_TE; ch++) {
        if ((uint32_t)s_channelPin[ch] != pin) continue;
        NRF_GPIOTE->INTENCLR       = (1u << ch);
        NRF_GPIOTE->CONFIG[ch]     = 0;
        NRF_GPIOTE->EVENTS_IN[ch]  = 0;
        s_channelPin[ch] = -1;
        s_callbacks[ch]  = nullptr;
        break;
    }
}
