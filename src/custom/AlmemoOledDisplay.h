#pragma once

#include "configuration.h"

#if defined(ALMEMO_OLED)

#include "AlmemoI2CClock.h"
#include "AlmemoMenu.h"
#include "AlmemoMenuLocal.h"
#include "AlmemoMenuRemote.h"
#include "AlmemoMenuRenderer.h"
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
 * Ist ein Receiver aufgesteckt, liefert AlmemoMenuRemote die zweite Quelle
 * (I2C statt lokal) und activeStruct() zeigt auf sie — Renderer und Controller
 * bleiben unberuehrt. Laut Konzept gilt: aufgesteckt = Display gehoert
 * ausschliesslich dem Receiver, zum Bedienen des Nodes wird er abgezogen.
 *
 * Noch nicht moeglich sind AENDERUNGEN am Receiver-Menue: dafuer fehlt der
 * Rueckweg (Edit als Write auf den Menue-Kanal). applyEdit() loggt das, statt
 * es stillschweigend zu verwerfen.
 *
 * ---------------------------------------------------------------------------
 * WICHTIG, I2C-Bustakt: Das Display haengt am selben Bus (globales `Wire`) wie
 * der ALMEMO-Sensor, und der laeuft auf ~50 kHz (Begruendung und Messwerte in
 * AlmemoI2CClock.h); 400 kHz sind wegen des Levelumsetzers auf der
 * Sensorleitung ohnehin ausgeschlossen.
 * SSD1306Wire ruft in connect() `Wire.setClock(_frequency)`
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

/* ALMEMO_I2C_FREQUENCY_REG und der Guard stehen in AlmemoI2CClock.h. */

/*
 * AlmemoSSD1306 — duenne Ableitung, nur um Wire.begin() einzufangen.
 *
 * OLEDDisplay::init() ruft ueber allocateBuffer() -> connect() unbedingt
 * Wire.begin(), und das setzt NRF_TWIM0->FREQUENCY auf K100 (Wire_nRF52.cpp:66).
 * connect() laeuft dabei VOR sendInitCommands(): ohne diesen Haken ginge die
 * komplette Init-Sequenz mit 100 kHz raus, also in dem Takt, in dem der Bus mit
 * gedocktem Receiver nachweislich kaum funktioniert (in I2C-Mitschnitten
 * antworten 0x3C und 0x50 dort zu 2 bzw. 6 %, bei ~50 kHz zu 100 %). Gehen die
 * Init-Kommandos verloren, bleibt der Controller unkonfiguriert und damit
 * dunkel, auch wenn spaeter alles mit 50 kHz ankommt — und neu initialisiert
 * wird nie, weil die Lib kein ACK prueft.
 *
 * Das Register wird hier DIREKT gesetzt und nicht ueber almemoI2cClockGuard():
 * an dieser Stelle ist die Korrektur erwartet und keine Warnung wert, sonst
 * waere die Warnung des Guards wertlos.
 */
class AlmemoSSD1306 : public SSD1306Wire
{
  public:
    using SSD1306Wire::SSD1306Wire;

    bool connect() override
    {
        const bool ok = SSD1306Wire::connect();
#if defined(ARCH_NRF52) && ALMEMO_I2C_FREQUENCY_REG
        NRF_TWIM0->FREQUENCY = ALMEMO_I2C_FREQUENCY_REG;
#endif
        return ok;
    }
};

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
        /* Gesetzt hat den Takt schon der connect()-Haken in AlmemoSSD1306, also
         * noch vor der Init-Sequenz. Hier nur die Bestaetigung ins Log — und
         * bewusst der tatsaechliche Registerinhalt statt des Defines, damit im
         * Boot-Log steht, was wirklich eingestellt ist. */
        const uint32_t freqReg = NRF_TWIM0->FREQUENCY;
        LOG_INFO("AlmemoOled: I2C FREQUENCY=0x%08lx (~%lu Hz)", (unsigned long)freqReg,
                 (unsigned long)((uint64_t)freqReg * 16000000ULL >> 32));
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
        /* Bustakt zurueckholen, falls ein Fremdtreiber Wire.begin() gerufen hat. */
        almemoI2cClockGuard();

        button.tick();

        const uint32_t now = millis();
        if (now - lastPollMs >= ALMEMO_OLED_POLL_MS) {
            lastPollMs = now;
            if (local.poll())
                dirty = true;

            /* Menue des aufgesteckten Receivers holen. Sobald eine gueltige
             * Struktur da ist, zeigt das Display sie statt der lokalen
             * (s. activeStruct()).
             *
             * Steckt nichts, kostet das einen NACKenden 2-Byte-Write pro
             * Intervall. Steckt etwas, kommt bei geaendertem Hash eine
             * BLOCKIERENDE Strukturuebertragung von ~295 ms dazu (s.
             * AlmemoMenuRemote.h) -- danach nur noch ~60 ms fuer die Werte. */
            if (remote.poll()) {
                dirty = true; /* Quellenwechsel und neue Werte muessen neu zeichnen */
                if (!remote.structure()) {
                    forceLocal = false; /* Abziehen stellt die Standardregel her */
                    LOG_INFO("AlmemoOled: Receiver-Menue weg");
                }
                else
                    /* Nur DEBUG: eine neue Struktur meldet AlmemoMenuRemote
                     * selbst mit INFO. Hier bleiben die Wertaenderungen uebrig,
                     * und die kommen im Anzeige-Intervall. */
                    LOG_DEBUG("AlmemoOled: Receiver-Werte aktualisiert (hash %08lx)",
                              (unsigned long)remote.hash());
            }
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

    /*
     * Aktive Quelle. Steckt ein Receiver, gehoert das Display AUSSCHLIESSLICH
     * ihm -- so ist das Konzept: zum Bedienen des Nodes wird der Receiver
     * abgezogen. Renderer und Controller merken keinen Unterschied, beide
     * sehen nur MenuStruct + MenuValue[].
     */
    /* Steckt ein Receiver, gehoert ihm das Display -- ausser der Benutzer hat im
     * Menue bewusst auf das Node-Display gewechselt. Die Umkehrung gibt es
     * nicht: ohne Receiver-Struktur ist nichts anzuzeigen. */
    bool              remoteActive() const { return remote.structure() != nullptr && !forceLocal; }
    /* BEIDE muessen ueber remoteActive() gehen. Fragt eines davon
     * remote.structure() direkt ab, ignoriert es forceLocal -- und dann zeigt
     * das Display die Receiver-Struktur mit den Node-Werten, also die
     * GW-Seiten mit ins Leere greifenden Wertindizes. */
    const MenuStruct *activeStruct() const
    {
        return remoteActive() ? remote.structure() : local.structure();
    }
    const MenuValue *activeValues() const { return remoteActive() ? remote.values() : local.values(); }

    /** Eine Aenderung an die aktive Quelle geben. */
    void applyEdit(const MenuEdit &edit)
    {
        /* Aktionen fuehrt immer das Geraet mit dem Display aus, egal aus welchem
         * Menue sie kommen -- sie gehen also NICHT an die Quelle. */
        const MenuStruct *s = activeStruct();
        if (s && edit.item < s->itemCount && (MenuItemKind)s->items[edit.item].kind == MenuItemKind::Action) {
            runMenuAction((uint8_t)edit.raw);
            return;
        }

        if (!remoteActive()) {
            local.apply(edit);
            return;
        }
        /* Noch kein Rueckweg: Edits muessten als Write auf den Menue-Kanal
         * gehen (itemIdx + raw, s. AlmemoMenuWire.h), und dafuer fehlt die
         * Gegenseite im Receiver. Bis dahin NICHT stillschweigend verwerfen --
         * sonst sieht es wie ein Bedienfehler aus. */
        LOG_WARN("AlmemoOled: Aenderung am Receiver-Menue noch nicht moeglich (Item %u, raw %ld)",
                 (unsigned)edit.item, (long)edit.raw);
    }

    /** MENU_ACTION_* ausfuehren. */
    void runMenuAction(uint8_t code)
    {
        switch (code) {
        case MENU_ACTION_SWAP_DISPLAY:
            if (!remote.structure()) {
                /* Vom Node-Menue aus gewaehlt, ohne dass ein Receiver steckt --
                 * dann gibt es nichts, worauf man wechseln koennte. */
                LOG_INFO("AlmemoOled: kein Receiver aufgesteckt, Anzeige bleibt beim Node");
                break;
            }
            forceLocal = !forceLocal;
            MenuController::reset(state);
            break;
        default:
            LOG_WARN("AlmemoOled: unbekannte Menue-Aktion %u", (unsigned)code);
            break;
        }
        dirty = true;
    }

    void onShort()
    {
        const MenuStruct *s = activeStruct();
        if (!s)
            return;
        MenuEdit edit{};
        if (MenuController::shortPress(*s, activeValues(), state, edit))
            applyEdit(edit);
        dirty = true;
    }

    void onLong()
    {
        const MenuStruct *s = activeStruct();
        if (!s)
            return;
        MenuEdit edit{};
        if (MenuController::longPress(*s, activeValues(), state, edit))
            applyEdit(edit);
        dirty = true;
    }

    void render()
    {
        const MenuStruct *s = activeStruct();
        if (!s) {
            MenuRenderer::renderWaiting(display, "warte auf Werte");
            return;
        }

        /* Quellenwechsel: Zustand zuruecksetzen. Die Seitenindizes der beiden
         * Menues bedeuten Verschiedenes, und ein offener Edit-Dialog gehoerte
         * zur alten Quelle -- stehenlassen waere schlimmer als ein Sprung auf
         * die erste Seite. */
        const bool rem = remoteActive();
        if (rem != lastRemoteActive) {
            lastRemoteActive = rem;
            MenuController::reset(state);
            LOG_INFO("AlmemoOled: Display zeigt jetzt das %s-Menue", rem ? "Receiver" : "Node");
        }

        /* Nach einem Strukturneubau kann der Zustand auf eine Seite zeigen,
         * die es nicht mehr gibt — dann zurueck auf den Anfang. */
        if (state.statusPage >= s->pageCount ||
            (MenuPageKind)s->pages[state.statusPage].kind != MenuPageKind::Status)
            MenuController::reset(state);

        MenuRenderer::render(display, *s, activeValues(), state);
    }

    AlmemoSSD1306 display{ALMEMO_OLED_ADDR, -1, -1, GEOMETRY_128_64, I2C_ONE, /* frequency */ -1};
    OneButton     button{ALMEMO_BUTTON_PIN, /* activeLow */ true, /* pullupActive */ true};

    AlmemoMenuLocal  local;
    AlmemoMenuRemote remote;
    /* Letzte aktive Quelle, fuer den Zustandsreset beim Wechsel. */
    bool lastRemoteActive = false;
    /* Benutzer hat im Menue auf das Node-Display gewechselt. Wird beim Abziehen
     * zurueckgesetzt: "aufgesteckt = Receiver" soll nach dem erneuten Anstecken
     * wieder gelten, sonst waere der Zustand unsichtbar und ueberraschend. */
    bool forceLocal = false;
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
