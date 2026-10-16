#pragma once

#include "configuration.h"

#if defined(ALMEMO_OLED)

#include "AlmemoMenu.h"
#include "AlmemoMenuLocal.h"
#include "concurrency/OSThread.h"

#include <OneButton.h>
#include <SSD1306Wire.h>

#if defined(ARCH_NRF52)
#include <nrf.h>
#endif

/*
 * AlmemoOledDisplay — Hardware-Schale um das generische Menue.
 *
 * Besitzt Display, Taster und Thread und verbindet sie mit den drei
 * geraeteunabhaengigen Teilen aus AlmemoMenu.h:
 *   MenuController  <- Tastendruecke
 *   MenuRenderer    -> Display
 *   AlmemoMenuLocal -- Daten dieses Nodes
 *
 * Wird spaeter ein Receiver aufgesteckt, kommt hier nur eine zweite Quelle
 * dazu (I2C statt lokal) und `src` zeigt auf sie — Renderer und Controller
 * bleiben unberuehrt. Laut Konzept gilt dann: aufgesteckt = Display gehoert
 * ausschliesslich dem Receiver, zum Bedienen des Nodes wird er abgezogen.
 *
 * ---------------------------------------------------------------------------
 * WICHTIG, I2C-Bustakt: Das Display haengt am selben Bus (globales `Wire`) wie
 * der ALMEMO-Sensor, und der Bus MUSS auf 100 kHz bleiben (Levelumsetzer auf
 * der Sensorleitung). SSD1306Wire ruft in connect() `Wire.setClock(_frequency)`
 * mit Default 700000, und Wire_nRF52 klemmt alles > 250k auf 400 kHz. Deshalb
 * steht der letzte Konstruktorparameter auf -1: dann ueberspringt connect()
 * das setClock komplett. Diesen Parameter NICHT entfernen.
 *
 * Kein Lock um die Bus-Zugriffe: alle OSThreads laufen kooperativ in derselben
 * Main-Loop-Task, Sensorlesen und Display-Schreiben koennen sich also nicht
 * verschraenken.
 * ---------------------------------------------------------------------------
 *
 * Taster: eigener Pin-Define, BEWUSST NICHT ueber BUTTON_PIN/UserButtonThread
 * — der wuerde denselben Pin belegen und bei langem Halten das Geraet
 * runterfahren (INPUT_BROKER_SHUTDOWN).
 */

#ifndef ALMEMO_OLED_ADDR
#define ALMEMO_OLED_ADDR 0x3C
#endif

#ifndef ALMEMO_BUTTON_PIN
#define ALMEMO_BUTTON_PIN 0 // D0
#endif

/* Taster-Entprellung: OneButton braucht regelmaessiges tick(). */
#ifndef ALMEMO_OLED_TICK_MS
#define ALMEMO_OLED_TICK_MS 20
#endif

/* So oft werden die laufenden Werte (Funk, System) eingesammelt. */
#ifndef ALMEMO_OLED_POLL_MS
#define ALMEMO_OLED_POLL_MS 500
#endif

/* Schwelle fuer "langer Druck", wie im Menuekonzept. */
#ifndef ALMEMO_OLED_LONGPRESS_MS
#define ALMEMO_OLED_LONGPRESS_MS 500
#endif

/* Panel um 180 Grad drehen; die ueblichen SSD1306-Module sitzen so.
 * Meshtastics graphics::Screen macht fuer OLEDs per Default dasselbe. */
#ifndef ALMEMO_OLED_FLIP
#define ALMEMO_OLED_FLIP 1
#endif

/*
 * Roh-Wert fuer NRF_TWIM0->FREQUENCY (Wire haengt auf TWIM0).
 *
 * Der ALMEMO-Logger faehrt den Bus mit 50 kHz, und der Bitbang-Slave des
 * Receivers ist darauf ausgelegt. Ueber die Arduino-API ist das nicht
 * erreichbar: Wire_nRF52::setClock() klemmt alles <= 100000 auf K100, und die
 * TWIM kennt offiziell nur K100/K250/K400.
 *
 * Das Register ist aber ein Frequenz-Skalierer mit f = FREQUENCY / 2^32 * 16 MHz
 * -- damit ergeben die dokumentierten Werte genau 100/250/400 kHz, und die
 * Haelfte von K100 (0x01980000) liefert ~49,8 kHz. Zwischenwerte sind
 * UNDOKUMENTIERT, funktionieren aber in der Praxis.
 *
 * Auf 0 setzen laesst den Bus bei den 100 kHz aus Wire.begin().
 */
#ifndef ALMEMO_I2C_FREQUENCY_REG
#define ALMEMO_I2C_FREQUENCY_REG 0x00CC0000UL /* ~49,8 kHz */
#endif

class AlmemoOledThread : public concurrency::OSThread
{
  public:
    AlmemoOledThread() : OSThread("AlmemoOled")
    {
        /* frequency = -1 -> connect() laesst setClock() aus (s. o.) */
        display.init();
#if ALMEMO_OLED_FLIP
        display.flipScreenVertically();
#endif
        display.displayOn();
        display.setFont(ArialMT_Plain_10);

#if defined(ARCH_NRF52) && ALMEMO_I2C_FREQUENCY_REG
        /* ZWINGEND NACH display.init(): SSD1306Wire::connect() ruft Wire.begin(),
         * und das setzt FREQUENCY wieder auf K100 (Wire_nRF52.cpp). Danach ruft
         * niemand mehr begin() -- initI2cIfNeccesary() haengt an _doI2cAutoInit,
         * und das ist false. */
        NRF_TWIM0->FREQUENCY = ALMEMO_I2C_FREQUENCY_REG;
        LOG_INFO("AlmemoOled: I2C FREQUENCY=0x%08lx (~%lu Hz)", (unsigned long)ALMEMO_I2C_FREQUENCY_REG,
                 (unsigned long)((uint64_t)ALMEMO_I2C_FREQUENCY_REG * 16000000ULL >> 32));
#endif

        button.setPressMs(ALMEMO_OLED_LONGPRESS_MS);
        button.setClickMs(250);
        button.attachClick(onShortStatic, this);
        button.attachLongPressStart(onLongStatic, this);

        /* Struktur sofort aufbauen, nicht erst beim ersten Senden: dann zeigt das
         * Display ab dem Boot die Messwerte-Seite mit "Kein Sensor" statt eines
         * Wartehinweises, und das Karussell ist von Anfang an bedienbar. */
        local.clear();

        render();
    }

    int32_t runOnce() override
    {
        button.tick();

        const uint32_t now = millis();
        if (now - lastPollMs >= ALMEMO_OLED_POLL_MS) {
            lastPollMs = now;
            if (local.poll())
                dirty = true;
        }

        if (dirty) {
            render();
            dirty = false;
        }
        return ALMEMO_OLED_TICK_MS;
    }

    /** Neue Messwerte vom Sender. */
    void publish(const AlmemoSensorPacket &pkt, const AlmemoMenuSensorInfo &info, uint32_t sendCount,
                 uint32_t intervalSecs)
    {
        local.publish(pkt, info, sendCount, intervalSecs);
        dirty = true;
    }

    /** Sensor weg. */
    void clear()
    {
        local.clear();
        dirty = true;
    }

  private:
    static void onShortStatic(void *ctx) { static_cast<AlmemoOledThread *>(ctx)->onShort(); }
    static void onLongStatic(void *ctx) { static_cast<AlmemoOledThread *>(ctx)->onLong(); }

    void onShort()
    {
        const MenuStruct *s = local.structure();
        if (!s)
            return;
        MenuEdit edit{};
        if (MenuController::shortPress(*s, local.values(), state, edit))
            local.apply(edit);
        dirty = true;
    }

    void onLong()
    {
        const MenuStruct *s = local.structure();
        if (!s)
            return;
        MenuEdit edit{};
        if (MenuController::longPress(*s, local.values(), state, edit))
            local.apply(edit);
        dirty = true;
    }

    void render()
    {
        const MenuStruct *s = local.structure();
        if (!s) {
            MenuRenderer::renderWaiting(display, "warte auf Werte");
            return;
        }
        /* Nach einem Strukturneubau kann der Zustand auf eine Seite zeigen,
         * die es nicht mehr gibt — dann zurueck auf den Anfang. */
        if (state.statusPage >= s->pageCount ||
            (MenuPageKind)s->pages[state.statusPage].kind != MenuPageKind::Status)
            MenuController::reset(state);

        MenuRenderer::render(display, *s, local.values(), state);
    }

    SSD1306Wire display{ALMEMO_OLED_ADDR, -1, -1, GEOMETRY_128_64, I2C_ONE, /* frequency */ -1};
    OneButton   button{ALMEMO_BUTTON_PIN, /* activeLow */ true, /* pullupActive */ true};

    AlmemoMenuLocal local;
    MenuState       state;
    bool            dirty      = false;
    uint32_t        lastPollMs = 0;
};

extern AlmemoOledThread *almemoOledThread;

/* Duenne Wrapper, damit der Sender nicht ueberall auf den Zeiger pruefen muss. */
void almemoOledPublish(const AlmemoSensorPacket &pkt, const AlmemoMenuSensorInfo &info, uint32_t sendCount,
                       uint32_t intervalSecs);
void almemoOledClear();

#endif // ALMEMO_OLED
