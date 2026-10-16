#pragma once

#include "configuration.h"

#if defined(ARCH_NRF52)
#include <nrf.h>
#endif

/*
 * AlmemoI2CClock — Bustakt des ALMEMO-I2C festnageln.
 *
 * Der ALMEMO-Bus ist ein 50-kHz-Bus: die Messgeraete/Logger fahren gemessene
 * 49,2 kHz, und der Bitbang-Slave des Receivers ist darauf ausgelegt. Seine
 * ISR braucht 3-4 us; bei 49 kHz hat sie pro SCL-Halbperiode 10,2 us Zeit und
 * antwortet in Mitschnitten zu 100 %, bei 100 kHz nur noch 5,0 us -- Quote 6 %.
 *
 * Roh-Wert fuer NRF_TWIM0->FREQUENCY (Wire haengt auf TWIM0):
 *
 * Ueber die Arduino-API ist 50 kHz nicht erreichbar -- Wire_nRF52::setClock()
 * klemmt alles <= 100000 auf K100, und die TWIM kennt offiziell nur
 * K100/K250/K400. Das Register ist aber ein Frequenz-Skalierer mit
 * f = FREQUENCY / 2^32 * 16 MHz; damit ergeben die dokumentierten Werte genau
 * 100/250/400 kHz. Zwischenwerte sind UNDOKUMENTIERT, funktionieren aber in der
 * Praxis (am Bus nachgemessen: 48,9 kHz).
 *
 * ACHTUNG, RASTER: Die TWIM uebernimmt FREQUENCY nicht bitgenau. Beobachtet:
 * 0x00CC0000 geschrieben, 0x00C80000 zurueckgelesen -- Bit 18 verworfen. Alle
 * dokumentierten Werte haben ihr niedrigstes gesetztes Bit auf Bit 19
 * (K100 = 0x01980000, K400 = 0x06680000), die Schrittweite ist also
 * 0x00080000 (~1953 Hz). Wer hier einen Wert ausserhalb des Rasters eintraegt,
 * bekommt ihn nie zurueckgelesen -- der Guard unten faengt das ab, aber der
 * Bus laeuft dann eben auf dem naechsten Rasterwert nach unten.
 *
 * Auf 0 setzen laesst den Bus bei den 100 kHz aus Wire.begin().
 *
 * ---------------------------------------------------------------------------
 * WARUM DER GUARD: Jedes Wire.begin() setzt FREQUENCY wieder auf K100
 * (Wire_nRF52.cpp:66). Das passiert nicht nur beim Start, sondern aus jeder
 * Fremdbibliothek heraus, die ihren Bus selbst initialisiert --
 * Adafruit_I2CDevice::begin() etwa ruft _wire->begin() bedingungslos, und damit
 * reisst ein einziger sht.begin() den Bustakt auf 100 kHz.
 *
 * Beobachtet: ein erfolgloser Erkennungslauf fiel bis zum SHT-Fallback durch,
 * danach lag der Bus dauerhaft auf 100 kHz -- und bei 100 kHz kann der Receiver
 * nicht mehr antworten, also scheitert die Erkennung weiter und faellt wieder
 * auf den SHT-Fallback. Selbstverriegelung.
 *
 * Der Guard macht die Invariante zu unserer eigenen Sache statt zu einer
 * Annahme ueber fremden Code: ein Registerlesen plus Vergleich, aufgerufen am
 * Eingang der Thread-Hauptschleifen. Er darf nur ZWISCHEN Transaktionen laufen,
 * nie mitten in einer -- das ist gegeben, weil alle OSThreads kooperativ in
 * derselben Main-Loop-Task haengen.
 * ---------------------------------------------------------------------------
 */

#ifndef ALMEMO_I2C_FREQUENCY_REG
#define ALMEMO_I2C_FREQUENCY_REG 0x00C80000UL /* ~48,8 kHz, im Raster (s. o.) */
#endif

/* Wer am Takt dreht, tut es meist in jedem Zyklus -- und der Guard sitzt in
 * einem 20-ms-Thread. Also hoechstens so oft eine Warnung. */
#ifndef ALMEMO_I2C_CLOCK_LOG_MS
#define ALMEMO_I2C_CLOCK_LOG_MS 5000
#endif

/**
 * FREQUENCY zuruecksetzen, falls jemand sie verstellt hat.
 * @return true wenn korrigiert werden musste.
 */
static inline bool almemoI2cClockGuard()
{
#if defined(ARCH_NRF52) && ALMEMO_I2C_FREQUENCY_REG
    /* Verglichen wird gegen den WIRKSAMEN Wert, nicht gegen das Define: die TWIM
     * kuerzt FREQUENCY aufs Raster (s. o.). Gegen das Define zu vergleichen
     * heisst, gegen die Hardware anzulaufen -- gemessen 50 Korrekturen pro
     * Sekunde, eine pro runOnce(). Der erste Aufruf lernt nur, was das Register
     * wirklich haelt, und warnt nicht. */
    static uint32_t effective = 0;

    const uint32_t cur = NRF_TWIM0->FREQUENCY;
    if (cur == effective)
        return false;

    NRF_TWIM0->FREQUENCY = ALMEMO_I2C_FREQUENCY_REG;

    if (effective == 0) {
        effective = NRF_TWIM0->FREQUENCY;
        return false;
    }

    static uint32_t lastLogMs = 0;
    static uint32_t fixes     = 0;
    const uint32_t  now       = millis();
    fixes++;
    if (lastLogMs == 0 || now - lastLogMs >= ALMEMO_I2C_CLOCK_LOG_MS) {
        lastLogMs = now ? now : 1;
        /* `effective` statt des Defines: geloggt wird, was im Register landet. */
        LOG_WARN("AlmemoI2C: FREQUENCY war 0x%08lx (~%lu Hz), zurueck auf 0x%08lx (%lu Korrekturen)",
                 (unsigned long)cur, (unsigned long)((uint64_t)cur * 16000000ULL >> 32),
                 (unsigned long)effective, (unsigned long)fixes);
    }
    return true;
#else
    return false;
#endif
}
