#pragma once

#include "AlmemoMenuWire.h"
#include "configuration.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/*
 * AlmemoMenu — geraeteunabhaengiges Menue fuer das 128x64-OLED.
 *
 * Diese Datei kennt KEIN Geraet: kein Sensor, kein Radio, keine Config. Sie
 * enthaelt nur
 *   - das Datenmodell (MenuStruct + MenuValue),
 *   - den Renderer (zeichnet eine Seite),
 *   - den Controller (Tastenlogik als Zustandsmaschine).
 * Wer die Daten liefert, steckt hinter MenuSource (siehe AlmemoMenuLocal.h).
 *
 * ---------------------------------------------------------------------------
 * WARUM REINE DATEN, OHNE FUNKTIONSPOINTER
 *
 * Ein aufgesteckter Receiver soll spaeter sein EIGENES Menue ueber I2C
 * herueberschicken und auf diesem Display bedient werden. Alles, was hier im
 * Modell steht, muss deshalb serialisierbar sein: POD, feste Groessen,
 * Index- statt Pointer-Referenzen, kein Heap. Dieselben Bytes im RAM wie auf
 * dem Draht. Ein Callback pro Eintrag (das Naheliegende) kann den Bus nicht
 * ueberqueren — daher die Trennung in Struktur und Werte.
 *
 * STRUKTUR SELTEN, WERTE LAUFEND
 *
 * MenuStruct (Seiten, Eintraege, Texte) aendert sich fast nie und traegt einen
 * Hash; der Empfaenger der Struktur holt sie nur neu, wenn der Hash sich
 * aendert. MenuValue[] ist der kleine Block, der dauernd aktualisiert wird.
 * Genau diese Trennung macht die I2C-Strecke spaeter bezahlbar.
 * ---------------------------------------------------------------------------
 *
 * Layout (Vorgabe aus dem Menuekonzept, ArialMT_Plain_10):
 *   y  0  Kopfzeile
 *   y 12  Trennlinie
 *   y 14/24/34/44/54  Zeile 1..5 (letzte Zeile oft Tastenhinweis)
 */

// =========================================================================
//  Grenzen
// =========================================================================

#ifndef MENU_MAX_PAGES
#define MENU_MAX_PAGES 10
#endif
#ifndef MENU_MAX_ITEMS
#define MENU_MAX_ITEMS 48
#endif
#ifndef MENU_MAX_VALUES
#define MENU_MAX_VALUES 40
#endif
#ifndef MENU_MAX_ENUMS
#define MENU_MAX_ENUMS 4
#endif
#ifndef MENU_MAX_OPTIONS
#define MENU_MAX_OPTIONS 32
#endif
#ifndef MENU_MAX_NUMBERS
#define MENU_MAX_NUMBERS 2
#endif
#ifndef MENU_POOL
#define MENU_POOL 640
#endif

#define MENU_ROWS 5 /* Inhaltszeilen pro Seite */

#define MENU_NO_IDX 0xFFu     /* uint8-Index "keiner" */
#define MENU_NO_TEXT 0xFFFFu  /* Pool-Offset "kein Text" */

/* Sentinel-Rohwerte in Auswahllisten (ausserhalb jedes echten Wertebereichs). */
#define MENU_OPT_MANUAL (INT32_MIN + 1)
#define MENU_OPT_CANCEL (INT32_MIN + 2)

/* Ziel einer Aenderung: das Geraet selbst und/oder der Node, auf dem es sitzt. */
#define MENU_TARGET_SELF 0x01
#define MENU_TARGET_HOST 0x02

/* Seiten-Flags */
#define MENU_PAGE_OPENS_MENU 0x01 /* langer Druck oeffnet page.target */

/* Item-Flags */
#define MENU_ITEM_DISABLED 0x01 /* nicht auswaehlbar (z.B. Preset bei use_preset=0) */

// =========================================================================
//  Modell — alles POD und serialisierbar
// =========================================================================

enum class MenuItemKind : uint8_t {
    Display,    /* nur anzeigen */
    EditEnum,   /* Auswahlliste, optional mit "Manuell" -> Ziffern-Eingabe */
    EditNumber, /* direkt Ziffern-Eingabe */
    Submenu,    /* oeffnet target */
    Back,       /* eine Ebene zurueck */
    Hint,       /* Tastenhinweis, linksbuendig, kein Wert */
};

enum class MenuPageKind : uint8_t {
    Status, /* Karussell-Seite: Label links, Wert rechts, kein Cursor */
    Menu,   /* Liste mit Cursor */
};

enum class MenuValueKind : uint8_t {
    None,     /* kein Wert (reines Label) */
    Int,      /* raw als Ganzzahl */
    Fixed,    /* raw * 10^exponent, Nachkommastellen = -exponent */
    Text,     /* text zeigt in den Pool */
    Duration, /* raw Sekunden, als "90s" / "1min 30s" / "1h 0min" */
    Invalid,  /* "----" : kein gueltiger Wert */
    Overflow, /* "OVR"  : ausserhalb des Messbereichs */
};

struct MenuItem { /* 10 Byte */
    uint16_t label;
    uint16_t unit;   /* Pool-Offset, MENU_NO_TEXT = keine Einheit */
    uint8_t  value;  /* Index in values[], MENU_NO_IDX = keiner */
    uint8_t  kind;   /* MenuItemKind */
    uint8_t  target; /* Submenu: Seite · EditEnum: Optionsliste */
    uint8_t  numSpec;/* EditEnum/EditNumber: Index in numbers[], sonst MENU_NO_IDX */
    uint8_t  flags;
    uint8_t  _pad;
};

struct MenuPage { /* 8 Byte */
    uint16_t title;
    uint8_t  firstItem;
    uint8_t  itemCount;
    uint8_t  kind; /* MenuPageKind */
    uint8_t  flags;
    uint8_t  target; /* MENU_PAGE_OPENS_MENU: zu oeffnende Menueseite */
    uint8_t  _pad;
};

struct MenuOption { /* 8 Byte */
    uint16_t label;
    int32_t  raw; /* Wert, oder MENU_OPT_MANUAL / MENU_OPT_CANCEL */
    uint16_t _pad;
};

struct MenuEnum { /* 2 Byte */
    uint8_t firstOption;
    uint8_t count;
};

struct MenuNumber { /* 12 Byte — Bereichspruefung der Ziffern-Eingabe */
    int32_t  min, max;
    uint8_t  digits; /* Anzahl Eingabestellen (<= 4) */
    uint8_t  _pad[3];
};

struct MenuValue { /* 8 Byte */
    int32_t  raw;
    uint16_t text;    /* Text: Pool-Offset */
    uint8_t  kind;    /* MenuValueKind */
    int8_t   exponent;/* Fixed */
};

struct MenuStruct {
    uint8_t  modelVersion;
    uint8_t  pageCount, itemCount, valueCount, enumCount, optionCount, numberCount;
    uint16_t deviceName; /* Pool-Offset: "A3F1" bzw. Receiver-Kennung */
    uint16_t poolLen;
    uint32_t hash; /* FNV-1a ueber Seiten/Items/Enums/Optionen/Pool */

    MenuPage   pages[MENU_MAX_PAGES];
    MenuItem   items[MENU_MAX_ITEMS];
    MenuEnum   enums[MENU_MAX_ENUMS];
    MenuOption options[MENU_MAX_OPTIONS];
    MenuNumber numbers[MENU_MAX_NUMBERS];
    char       pool[MENU_POOL];

    const char *text(uint16_t off) const { return (off == MENU_NO_TEXT || off >= poolLen) ? "" : &pool[off]; }
};

// =========================================================================
//  Builder — fuellt eine MenuStruct auf
// =========================================================================

class MenuBuilder
{
    MenuStruct &m;
    uint8_t     curPage   = MENU_NO_IDX;
    uint8_t     curEnum   = MENU_NO_IDX;

  public:
    explicit MenuBuilder(MenuStruct &s) : m(s) { reset(); }

    void reset()
    {
        memset(&m, 0, sizeof(m));
        m.modelVersion = ALMEMO_MENU_MODEL_VERSION; /* eine Quelle: AlmemoMenuWire.h */
        m.deviceName   = MENU_NO_TEXT;
        curPage = curEnum = MENU_NO_IDX;
    }

    /** Text in den Pool legen; MENU_NO_TEXT wenn kein Platz oder nullptr. */
    uint16_t str(const char *s)
    {
        if (!s)
            return MENU_NO_TEXT;
        const size_t len = strlen(s);
        if (m.poolLen + len + 1 > MENU_POOL)
            return MENU_NO_TEXT;
        const uint16_t off = m.poolLen;
        memcpy(&m.pool[off], s, len + 1);
        m.poolLen = (uint16_t)(off + len + 1);
        return off;
    }

    void deviceName(const char *s) { m.deviceName = str(s); }

    /** Neue Seite anlegen; folgende item()-Aufrufe gehoeren zu ihr. */
    uint8_t page(const char *title, MenuPageKind kind, uint8_t flags = 0, uint8_t target = MENU_NO_IDX)
    {
        if (m.pageCount >= MENU_MAX_PAGES)
            return MENU_NO_IDX;
        const uint8_t idx = m.pageCount++;
        MenuPage &p       = m.pages[idx];
        p.title           = str(title);
        p.firstItem       = m.itemCount;
        p.itemCount       = 0;
        p.kind            = (uint8_t)kind;
        p.flags           = flags;
        p.target          = target;
        curPage           = idx;
        return idx;
    }

    uint8_t item(const char *label, uint8_t valueIdx = MENU_NO_IDX, const char *unit = nullptr,
                 MenuItemKind kind = MenuItemKind::Display, uint8_t target = MENU_NO_IDX,
                 uint8_t numSpec = MENU_NO_IDX, uint8_t flags = 0)
    {
        if (m.itemCount >= MENU_MAX_ITEMS || curPage == MENU_NO_IDX)
            return MENU_NO_IDX;
        const uint8_t idx = m.itemCount++;
        MenuItem &it      = m.items[idx];
        it.label          = str(label);
        it.unit           = unit ? str(unit) : MENU_NO_TEXT;
        it.value          = valueIdx;
        it.kind           = (uint8_t)kind;
        it.target         = target;
        it.numSpec        = numSpec;
        it.flags          = flags;
        m.pages[curPage].itemCount++;
        return idx;
    }

    uint8_t beginEnum()
    {
        if (m.enumCount >= MENU_MAX_ENUMS)
            return MENU_NO_IDX;
        curEnum                          = m.enumCount++;
        m.enums[curEnum].firstOption     = m.optionCount;
        m.enums[curEnum].count           = 0;
        return curEnum;
    }

    /** @return Pool-Offset des Labels — so muss der Text nicht zweimal abgelegt werden. */
    uint16_t option(const char *label, int32_t raw)
    {
        if (m.optionCount >= MENU_MAX_OPTIONS || curEnum == MENU_NO_IDX)
            return MENU_NO_TEXT;
        MenuOption &o = m.options[m.optionCount++];
        o.label       = str(label);
        o.raw         = raw;
        m.enums[curEnum].count++;
        return o.label;
    }

    uint8_t number(int32_t min, int32_t max, uint8_t digits)
    {
        if (m.numberCount >= MENU_MAX_NUMBERS)
            return MENU_NO_IDX;
        const uint8_t idx = m.numberCount++;
        m.numbers[idx]    = MenuNumber{min, max, digits, {0, 0, 0}};
        return idx;
    }

    /** Hash bilden. Danach ist die Struktur fertig. */
    void finish()
    {
        uint32_t h = 2166136261u; /* FNV-1a, reine Aenderungserkennung */
        auto feed  = [&h](const void *p, size_t n) {
            const uint8_t *b = (const uint8_t *)p;
            for (size_t i = 0; i < n; i++) {
                h ^= b[i];
                h *= 16777619u;
            }
        };
        feed(m.pages, sizeof(MenuPage) * m.pageCount);
        feed(m.items, sizeof(MenuItem) * m.itemCount);
        feed(m.enums, sizeof(MenuEnum) * m.enumCount);
        feed(m.options, sizeof(MenuOption) * m.optionCount);
        feed(m.numbers, sizeof(MenuNumber) * m.numberCount);
        feed(m.pool, m.poolLen);
        m.hash = h;
    }
};

// =========================================================================
//  Formatierung
// =========================================================================

namespace menufmt
{

/** Einheit druckbar machen: ALMEMO liefert Grad als 0xF8, die OLED-Lib
 *  verwirft aber jedes rohe Byte >= 128, dem kein 0xC2/0xC3 vorangeht
 *  (DefaultFontTableLookup). Also die UTF-8-Folge fuer U+00B0 schreiben. */
inline void unit(const char *in, char *out, size_t cap)
{
    size_t n = 0;
    for (size_t i = 0; in && in[i] && i < 2; i++) {
        const unsigned char c = (unsigned char)in[i];
        if (c == ' ')
            continue;
        if (c == 0xF8) {
            if (n + 3 > cap)
                break;
            out[n++] = '\xC2';
            out[n++] = '\xB0';
        } else {
            if (n + 2 > cap)
                break;
            out[n++] = (char)c;
        }
    }
    out[n] = '\0';
}

/** Sekunden lesbar: "45s", "1min 30s", "1h 0min". */
inline void duration(int32_t secs, char *out, size_t cap)
{
    if (secs < 0)
        secs = 0;
    if (secs < 60)
        snprintf(out, cap, "%lds", (long)secs);
    else if (secs < 3600)
        snprintf(out, cap, "%ldmin %lds", (long)(secs / 60), (long)(secs % 60));
    else
        snprintf(out, cap, "%ldh %ldmin", (long)(secs / 3600), (long)((secs % 3600) / 60));
}

/** Festkommawert aus Rohwert und Exponent — bewusst NICHT aus einem float. */
inline void fixed(int32_t raw, int8_t exponent, char *out, size_t cap)
{
    int dec = -exponent;
    if (dec < 0)
        dec = 0;
    if (dec > 4)
        dec = 4;
    if (dec == 0) {
        snprintf(out, cap, "%ld", (long)raw);
        return;
    }
    int32_t div = 1;
    for (int i = 0; i < dec; i++)
        div *= 10;
    const bool    neg = raw < 0;
    const int32_t a   = neg ? -raw : raw;
    snprintf(out, cap, "%s%ld.%0*ld", neg ? "-" : "", (long)(a / div), dec, (long)(a % div));
}

/** Einen Wert in Text wandeln (ohne Einheit). */
inline void value(const MenuStruct &s, const MenuValue &v, char *out, size_t cap)
{
    switch ((MenuValueKind)v.kind) {
    case MenuValueKind::Int:
        snprintf(out, cap, "%ld", (long)v.raw);
        break;
    case MenuValueKind::Fixed:
        fixed(v.raw, v.exponent, out, cap);
        break;
    case MenuValueKind::Duration:
        duration(v.raw, out, cap);
        break;
    case MenuValueKind::Text:
        snprintf(out, cap, "%s", s.text(v.text));
        break;
    case MenuValueKind::Invalid:
        snprintf(out, cap, "----");
        break;
    case MenuValueKind::Overflow:
        snprintf(out, cap, "OVR");
        break;
    default:
        out[0] = '\0';
        break;
    }
}

} // namespace menufmt

// =========================================================================
//  Zustand und Controller
// =========================================================================

enum class MenuMode : uint8_t {
    Carousel,   /* Status-Seiten durchblaettern */
    Menu,       /* Liste mit Cursor */
    SelectList, /* Auswahlliste einer Einstellung */
    DigitEdit,  /* Ziffern-Eingabe */
    Confirm,    /* Bestaetigung */
    Invalid,    /* Wert ausserhalb des Bereichs */
};

enum class MenuAction : uint8_t { Apply, ApplyBoth, Correct, Cancel };

struct MenuEdit {
    uint8_t item;    /* Index in items[] */
    int32_t raw;     /* neuer Wert */
    uint8_t targets; /* MENU_TARGET_SELF | MENU_TARGET_HOST */
};

/**
 * Sichtzustand. Bewusst getrennt von der Struktur: beim Umschalten des
 * Geraets (Receiver auf-/abgesteckt) wird nur der Zustand zurueckgesetzt.
 */
struct MenuState {
    MenuMode mode = MenuMode::Carousel;

    uint8_t statusPage = 0;           /* aktuelle Karussell-Seite */
    uint8_t menuPage   = MENU_NO_IDX; /* aktuelle Menueseite */
    uint8_t cursor     = 0;
    uint8_t scroll     = 0;

    uint8_t editItem = MENU_NO_IDX;
    uint8_t editEnum = MENU_NO_IDX;
    uint8_t numSpec  = MENU_NO_IDX;

    uint8_t digits[4]  = {0, 0, 0, 0};
    uint8_t digitCount = 4;
    uint8_t activeDigit = 0;

    int32_t candidate = 0;
    bool    fromDigits = false; /* Confirm: "Korrigieren" anbieten */
    bool    offerBoth  = false; /* Confirm: "Auf beide anwenden" anbieten */

    MenuAction actions[4] = {};
    uint8_t    actionCount = 0;
    uint8_t    actionCursor = 0;

    char message[28] = {}; /* Text fuer den Invalid-Screen */
};

/**
 * Tastenlogik. Kennt nur Struktur, Werte und Zustand — kein Display, kein
 * Geraet. Ein Tastendruck kann eine Aenderung erzeugen; die fuehrt der
 * Aufrufer aus (lokal schreiben oder per I2C schicken).
 */
class MenuController
{
  public:
    /** true = es liegt eine auszufuehrende Aenderung in `out`. */
    static bool shortPress(const MenuStruct &s, const MenuValue *vals, MenuState &st, MenuEdit &out)
    {
        (void)vals;
        (void)out;
        switch (st.mode) {
        case MenuMode::Carousel:
            st.statusPage = nextStatusPage(s, st.statusPage);
            break;
        case MenuMode::Menu: {
            const MenuPage &p = s.pages[st.menuPage];
            if (p.itemCount)
                st.cursor = (uint8_t)((st.cursor + 1) % p.itemCount);
            clampScroll(st, p.itemCount);
            break;
        }
        case MenuMode::SelectList: {
            const MenuEnum &e = s.enums[st.editEnum];
            if (e.count)
                st.cursor = (uint8_t)((st.cursor + 1) % e.count);
            clampScroll(st, e.count);
            break;
        }
        case MenuMode::DigitEdit:
            st.digits[st.activeDigit] = (uint8_t)((st.digits[st.activeDigit] + 1) % 10);
            break;
        case MenuMode::Confirm:
        case MenuMode::Invalid:
            if (st.actionCount)
                st.actionCursor = (uint8_t)((st.actionCursor + 1) % st.actionCount);
            break;
        }
        return false;
    }

    /** true = es liegt eine auszufuehrende Aenderung in `out`. */
    static bool longPress(const MenuStruct &s, const MenuValue *vals, MenuState &st, MenuEdit &out)
    {
        switch (st.mode) {
        case MenuMode::Carousel: {
            const MenuPage &p = s.pages[st.statusPage];
            if ((p.flags & MENU_PAGE_OPENS_MENU) && p.target != MENU_NO_IDX) {
                st.mode     = MenuMode::Menu;
                st.menuPage = p.target;
                st.cursor = st.scroll = 0;
            }
            break;
        }
        case MenuMode::Menu:
            return activateMenuItem(s, vals, st);
        case MenuMode::SelectList:
            return activateOption(s, st);
        case MenuMode::DigitEdit:
            st.activeDigit++;
            if (st.activeDigit >= st.digitCount) {
                int32_t v = 0;
                for (uint8_t i = 0; i < st.digitCount; i++)
                    v = v * 10 + st.digits[i];
                st.candidate  = v;
                st.fromDigits = true;
                enterConfirmOrInvalid(s, st);
            }
            break;
        case MenuMode::Confirm:
        case MenuMode::Invalid:
            return runAction(s, st, out);
        }
        return false;
    }

    /** Nach einem Geraetewechsel oder Strukturwechsel aufrufen. */
    static void reset(MenuState &st)
    {
        st = MenuState{};
    }

  private:
    static uint8_t nextStatusPage(const MenuStruct &s, uint8_t from)
    {
        for (uint8_t i = 1; i <= s.pageCount; i++) {
            const uint8_t cand = (uint8_t)((from + i) % s.pageCount);
            if ((MenuPageKind)s.pages[cand].kind == MenuPageKind::Status)
                return cand;
        }
        return from;
    }

    static void clampScroll(MenuState &st, uint8_t count)
    {
        /* Cursor im Fenster halten; letzte Zeile bleibt fuer den Hinweis frei. */
        const uint8_t window = MENU_ROWS - 1;
        if (count <= window) {
            st.scroll = 0;
            return;
        }
        if (st.cursor < st.scroll)
            st.scroll = st.cursor;
        else if (st.cursor >= st.scroll + window)
            st.scroll = (uint8_t)(st.cursor - window + 1);
    }

    static bool activateMenuItem(const MenuStruct &s, const MenuValue *vals, MenuState &st)
    {
        const MenuPage &p = s.pages[st.menuPage];
        if (!p.itemCount)
            return false;
        const uint8_t  idx = (uint8_t)(p.firstItem + st.cursor);
        const MenuItem &it = s.items[idx];

        if (it.flags & MENU_ITEM_DISABLED)
            return false;

        switch ((MenuItemKind)it.kind) {
        case MenuItemKind::Back:
            st.mode   = MenuMode::Carousel;
            st.cursor = st.scroll = 0;
            break;
        case MenuItemKind::Submenu:
            if (it.target != MENU_NO_IDX) {
                st.menuPage = it.target;
                st.cursor = st.scroll = 0;
            }
            break;
        case MenuItemKind::EditEnum:
            st.editItem = idx;
            st.editEnum = it.target;
            st.numSpec  = it.numSpec;
            st.mode     = MenuMode::SelectList;
            st.cursor   = currentOption(s, vals, it);
            st.scroll   = 0;
            clampScroll(st, s.enums[st.editEnum].count);
            break;
        case MenuItemKind::EditNumber:
            st.editItem = idx;
            st.editEnum = MENU_NO_IDX;
            st.numSpec  = it.numSpec;
            startDigits(s, vals, st, it);
            break;
        default:
            break;
        }
        return false;
    }

    /** Index der Option, die dem aktuellen Wert entspricht (sonst 0). */
    static uint8_t currentOption(const MenuStruct &s, const MenuValue *vals, const MenuItem &it)
    {
        if (it.value == MENU_NO_IDX || !vals)
            return 0;
        const MenuEnum &e   = s.enums[it.target];
        const int32_t   cur = vals[it.value].raw;
        for (uint8_t i = 0; i < e.count; i++)
            if (s.options[e.firstOption + i].raw == cur)
                return i;
        return 0;
    }

    static void startDigits(const MenuStruct &s, const MenuValue *vals, MenuState &st, const MenuItem &it)
    {
        uint8_t digits = 4;
        if (st.numSpec != MENU_NO_IDX && s.numbers[st.numSpec].digits <= 4)
            digits = s.numbers[st.numSpec].digits;
        st.digitCount = digits ? digits : 4;

        int32_t cur = 0;
        if (it.value != MENU_NO_IDX && vals)
            cur = vals[it.value].raw;
        if (cur < 0)
            cur = 0;

        for (int i = (int)st.digitCount - 1; i >= 0; i--) {
            st.digits[i] = (uint8_t)(cur % 10);
            cur /= 10;
        }
        st.activeDigit = 0;
        st.mode        = MenuMode::DigitEdit;
    }

    static bool activateOption(const MenuStruct &s, MenuState &st)
    {
        const MenuEnum   &e = s.enums[st.editEnum];
        if (!e.count)
            return false;
        const MenuOption &o = s.options[e.firstOption + st.cursor];

        if (o.raw == MENU_OPT_CANCEL) {
            st.mode   = MenuMode::Menu;
            st.cursor = st.scroll = 0;
            return false;
        }
        if (o.raw == MENU_OPT_MANUAL) {
            const MenuItem &it = s.items[st.editItem];
            startDigits(s, nullptr, st, it); /* ohne vals: ab 0 */
            return false;
        }
        st.candidate  = o.raw;
        st.fromDigits = false;
        enterConfirmOrInvalid(s, st);
        return false;
    }

    static void enterConfirmOrInvalid(const MenuStruct &s, MenuState &st)
    {
        if (st.numSpec != MENU_NO_IDX) {
            const MenuNumber &n = s.numbers[st.numSpec];
            if (st.candidate < n.min || st.candidate > n.max) {
                snprintf(st.message, sizeof(st.message), "%ld: nur %ld-%ld!", (long)st.candidate, (long)n.min,
                         (long)n.max);
                st.mode         = MenuMode::Invalid;
                st.actionCount  = 0;
                st.actions[st.actionCount++] = MenuAction::Correct;
                st.actions[st.actionCount++] = MenuAction::Cancel;
                st.actionCursor = 0;
                return;
            }
        }
        st.mode        = MenuMode::Confirm;
        st.actionCount = 0;
        st.actions[st.actionCount++] = MenuAction::Apply;
        if (st.offerBoth)
            st.actions[st.actionCount++] = MenuAction::ApplyBoth;
        if (st.fromDigits)
            st.actions[st.actionCount++] = MenuAction::Correct;
        st.actions[st.actionCount++] = MenuAction::Cancel;
        st.actionCursor = 0;
    }

    static bool runAction(const MenuStruct &s, MenuState &st, MenuEdit &out)
    {
        if (!st.actionCount)
            return false;
        switch (st.actions[st.actionCursor]) {
        case MenuAction::Apply:
        case MenuAction::ApplyBoth:
            out.item    = st.editItem;
            out.raw     = st.candidate;
            out.targets = (uint8_t)(MENU_TARGET_SELF |
                                    (st.actions[st.actionCursor] == MenuAction::ApplyBoth ? MENU_TARGET_HOST : 0));
            st.mode     = MenuMode::Menu;
            st.cursor = st.scroll = 0;
            return true;
        case MenuAction::Correct: {
            const MenuItem &it = s.items[st.editItem];
            (void)it;
            st.activeDigit = 0; /* zurueck auf die erste Stelle, Wert bleibt */
            st.mode        = MenuMode::DigitEdit;
            return false;
        }
        case MenuAction::Cancel:
        default:
            st.mode   = MenuMode::Menu;
            st.cursor = st.scroll = 0;
            return false;
        }
    }
};
