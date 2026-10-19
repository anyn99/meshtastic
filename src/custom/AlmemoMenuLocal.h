#pragma once

#include "configuration.h"

#if defined(ALMEMO_OLED)

#include "AlmemoCommon.h"
#include "AlmemoMenu.h"
#include "AlmemoMenuSettings.h"
#include "DisplayFormatters.h"
#include "NodeDB.h"
#include "PowerStatus.h"
#include "power/PowerHAL.h"
#include "airtime.h"

/*
 * AlmemoMenuLocal — die Menuequelle dieses Nodes.
 *
 * Baut die MenuStruct dieses Geraets und fuellt die Werte. Das Gegenstueck
 * kommt spaeter: eine Quelle, die Struktur und Werte eines aufgesteckten
 * Receivers ueber I2C liest. Renderer und Controller sehen beide gleich.
 *
 * Struktur wird NUR neu gebaut, wenn sich der Fingerabdruck aendert (Quelle,
 * Kennung, Kanalzahl, Slot-Belegung, Einheiten) — nicht bei jedem Messwert.
 * Alle Texte, die zur Laufzeit wechseln koennen (Preset-Namen, Statustexte,
 * "USB versorgt"), liegen schon beim Bauen im Pool; der Wert zeigt dann nur
 * auf den passenden Offset. Deshalb braucht ein Preset-Wechsel keinen Neubau.
 */

/* Spiegelt den Default des Senders; gleicher #ifndef-Guard, also derselbe
 * Wert auch wenn das Env ihn ueberschreibt. */

/** Was der Sender ueber seine aktuelle Quelle mitteilt. */
struct AlmemoMenuSensorInfo {
    const char *source      = "NONE";
    const char *name        = "";
    const char *version     = "";
    uint8_t     typeByte    = ALMEMO_TYPE_NONE; /* ALMEMO-Kennung des Fuehlers */
    uint8_t     i2cAddr     = 0;                /* nur bei ALMEMO_TYPE_I2C */
    uint8_t     channels    = 0;                /* aktive Messkanaele */
    uint8_t     channelsMax = 0;                /* 0 = ohne "/max" anzeigen */
    uint8_t     status      = 1;                /* Index in STATUS_TEXT */
};

class AlmemoMenuLocal
{
  public:
    /* Feste Werteplaetze — der Builder verdrahtet die Items darauf. */
    enum : uint8_t {
        VAL_MEAS0 = 0, /* 0..9: Messwerte */
        VAL_TYPE  = 10,
        VAL_IDENT,
        VAL_VER,
        VAL_CHANS,
        VAL_STATUS,
        VAL_PACKETS,
        VAL_CHUTIL,
        VAL_AIRUTIL,
        VAL_SINCE,
        VAL_BATT,
        VAL_FIRMWARE,
        VAL_INTERVAL,
        VAL_PRESET,
        VAL_NEXT, /* Sekunden bis zur naechsten Aktualisierung */
        VAL_COUNT
    };
    static constexpr uint8_t MAX_MEAS = 10;

    /** Fuehlertyp fuer die Sensor-Seite; bei direktem I2C-Sensor mit Adresse. */
    static void typeText(const AlmemoMenuSensorInfo &i, char *out, size_t cap)
    {
        switch (i.typeByte) {
        case ALMEMO_TYPE_ANALOG:
            snprintf(out, cap, "ANALOG");
            break;
        case ALMEMO_TYPE_D6:
            snprintf(out, cap, "D6");
            break;
        case ALMEMO_TYPE_D7:
            snprintf(out, cap, "D7");
            break;
        case ALMEMO_TYPE_I2C:
            snprintf(out, cap, "I2C 0x%02X", i.i2cAddr);
            break;
        case ALMEMO_TYPE_EMU:
            snprintf(out, cap, "Emulator");
            break;
        default:
            snprintf(out, cap, "-"); /* nichts gesteckt */
            break;
        }
    }

    /** Kanalzahl: "2/4" wenn es eine Slot-Obergrenze gibt, sonst nur die Anzahl. */
    static void channelsText(const AlmemoMenuSensorInfo &i, char *out, size_t cap)
    {
        if (i.typeByte == ALMEMO_TYPE_NONE)
            snprintf(out, cap, "-");
        else if (i.channelsMax)
            snprintf(out, cap, "%u/%u", (unsigned)i.channels, (unsigned)i.channelsMax);
        else
            snprintf(out, cap, "%u", (unsigned)i.channels);
    }

    static constexpr const char *STATUS_TEXT[] = {"OK", "kein Sensor", "Timeout", "Überlauf"};
    static constexpr uint8_t     STATUS_COUNT  = 4;

    const MenuStruct *structure() const { return built ? &s : nullptr; }
    const MenuValue  *values() const { return vals; }

    /** Neue Messwerte uebernehmen; baut die Struktur neu, falls noetig. */
    void publish(const AlmemoSensorPacket &pkt, const AlmemoMenuSensorInfo &info, uint32_t sendCount,
                 uint32_t intervalSecs)
    {
        const uint32_t fp = fingerprint(pkt, info);
        if (!built || fp != lastFingerprint) {
            rebuild(pkt, info);
            lastFingerprint = fp;
        }

        const uint8_t n = pkt.count > MAX_MEAS ? MAX_MEAS : pkt.count;
        for (uint8_t i = 0; i < n; i++) {
            MenuValue &v = vals[VAL_MEAS0 + i];
            v.raw        = pkt.values[i].raw;
            v.exponent   = pkt.values[i].exponent;
            v.kind       = (uint8_t)MenuValueKind::Fixed;
        }
        for (uint8_t i = n; i < MAX_MEAS; i++)
            vals[VAL_MEAS0 + i].kind = (uint8_t)MenuValueKind::Invalid;

        /* Beides ueberlebt rebuild() absichtlich als Member: clear() nullt die
         * Werte, und der Paketzaehler darf beim Sensorverlust nicht auf 0 fallen. */
        lastPackets        = sendCount;
        pushedIntervalSecs = intervalSecs;
        lastSendMs         = millis();
        poll();
    }

    /** Sensor weg: Werte verwerfen, Struktur auf "Keine Messstellen" stellen. */
    void clear()
    {
        AlmemoSensorPacket   empty = {};
        AlmemoMenuSensorInfo info;

        /* Gleicher Schutz wie in publish(): steht die Struktur schon im
         * Leer-Zustand, muss sie nicht neu gebaut werden. clear() kommt bei
         * jedem Disconnect, und ein Rebuild zieht den kompletten Pool neu.
         *
         * Ueberspringen ist gefahrlos: die Leer-Struktur hat gar keine
         * Messwert-Items, alte vals[] werden also nicht gerendert -- und wenn
         * sie schon stand, wurden sie beim vorigen clear() ohnehin verworfen. */
        const uint32_t fp = fingerprint(empty, info);
        if (!built || fp != lastFingerprint) {
            rebuild(empty, info);
            lastFingerprint = fp;
        }
        poll();
    }

    /**
     * Laufende Werte einsammeln (Funk, System, Einstellungen).
     * @return true wenn sich etwas geaendert hat -> neu zeichnen.
     */
    bool poll()
    {
        if (!built)
            return false;
        MenuValue before[VAL_COUNT];
        memcpy(before, vals, sizeof(before));

        if (airTime) {
            setFixed(VAL_CHUTIL, (int32_t)(airTime->channelUtilizationPercent() * 10.0f), -1);
            setFixed(VAL_AIRUTIL, (int32_t)(airTime->utilizationTXPercent() * 10.0f), -1);
        }

        /* Paketzaehler aus dem Member, nicht aus dem genullten Wertearray. */
        vals[VAL_PACKETS].kind = (uint8_t)MenuValueKind::Int;
        vals[VAL_PACKETS].raw  = (int32_t)lastPackets;

        vals[VAL_SINCE].raw  = lastSendMs ? (int32_t)((millis() - lastSendMs) / 1000) : 0;
        vals[VAL_SINCE].kind = lastSendMs ? (uint8_t)MenuValueKind::Int : (uint8_t)MenuValueKind::Invalid;

        /* Intervall: solange noch nie gesendet wurde (z.B. ohne Sensor) aus der
         * Config ableiten, sonst stand hier 0s. 0 in der Config heisst Default.
         * Nach dem ersten Senden gilt der vom Sender gemeldete Wert; der kann bei
         * einer anderen Rolle als SENSOR mit der Knotenzahl skaliert sein. */
        {
            uint32_t iv = pushedIntervalSecs;
            if (!iv) {
                const uint32_t cfg = moduleConfig.telemetry.environment_update_interval;
                iv                 = cfg ? cfg : ALMEMO_SENDER_DEFAULT_INTERVAL_SECS;
            }
            vals[VAL_INTERVAL].kind = (uint8_t)MenuValueKind::Duration;
            vals[VAL_INTERVAL].raw  = (int32_t)iv;
        }

        /* Restzeit bis zur naechsten Aktualisierung; vor dem ersten Senden "----". */
        if (lastSendMs) {
            int32_t rest = vals[VAL_INTERVAL].raw - (int32_t)((millis() - lastSendMs) / 1000);
            if (rest < 0)
                rest = 0;
            vals[VAL_NEXT].kind = (uint8_t)MenuValueKind::Int;
            vals[VAL_NEXT].raw  = rest;
        } else {
            vals[VAL_NEXT].kind = (uint8_t)MenuValueKind::Invalid;
        }

        /* Akku: bei USB-Versorgung Text statt Spannung (die Einheit wird dann nicht
         * angehaengt). Erkannt wird das wie beim Stromsparen des Senders ueber
         * powerHAL_isVBUSConnected() -- powerStatus->getHasUSB() war hier
         * unzuverlaessig und zeigte weiter mV. */
        if (powerHAL_isVBUSConnected()) {
            vals[VAL_BATT].kind = (uint8_t)MenuValueKind::Text;
            vals[VAL_BATT].text = offUsb;
        } else if (powerStatus) {
            vals[VAL_BATT].kind = (uint8_t)MenuValueKind::Int;
            vals[VAL_BATT].raw  = powerStatus->getBatteryVoltageMv();
        }

        /* Preset: Text auf den passenden Pool-Offset zeigen lassen. */
        vals[VAL_PRESET].kind = (uint8_t)MenuValueKind::Text;
        vals[VAL_PRESET].text = presetTextOffset();
        vals[VAL_PRESET].raw  = (int32_t)config.lora.modem_preset;

        return memcmp(before, vals, sizeof(before)) != 0;
    }

    /** Eine Aenderung aus dem Menue ausfuehren. */
    bool apply(const MenuEdit &e)
    {
        if (!built || e.item >= s.itemCount)
            return false;
        if (e.item == itemInterval) {
            moduleConfig.telemetry.environment_update_interval = (uint32_t)e.raw;
            nodeDB->saveToDisk(SEGMENT_MODULECONFIG);
            /* Zurueck auf "aus der Config ableiten", sonst zeigte die Anzeige
             * bis zum naechsten Senden weiter das alte Intervall. */
            pushedIntervalSecs = 0;
            LOG_INFO("AlmemoMenu: Sendeintervall -> %lds", (long)e.raw);
            return true;
        }
        if (e.item == itemPreset) {
            config.lora.modem_preset = (meshtastic_Config_LoRaConfig_ModemPreset)e.raw;
            config.lora.use_preset   = true;
            nodeDB->saveToDisk(SEGMENT_CONFIG);
            LOG_INFO("AlmemoMenu: Preset -> %s",
                     DisplayFormatters::getModemPresetDisplayName(config.lora.modem_preset, false, true));
            return true;
        }
        return false;
    }

    /** Item-Index der Preset-Einstellung (fuer "Auf beide anwenden"). */
    uint8_t presetItem() const { return itemPreset; }

  private:
    MenuStruct s{};
    MenuValue  vals[VAL_COUNT]{};
    bool       built           = false;
    uint32_t   lastFingerprint = 0;
    uint32_t   lastSendMs      = 0;
    uint32_t   pushedIntervalSecs = 0; /* 0 = noch nichts gesendet */
    uint32_t   lastPackets        = 0; /* ueberlebt rebuild() */

    uint8_t  itemInterval = MENU_NO_IDX;
    uint8_t  itemPreset   = MENU_NO_IDX;
    uint16_t offUsb       = MENU_NO_TEXT;
    uint16_t offPreset[16]{};  /* Pool-Offset je Preset-Enumwert */
    uint16_t offCustom    = MENU_NO_TEXT;
    uint16_t offStatus[STATUS_COUNT]{};
    uint16_t offType      = MENU_NO_TEXT;
    uint16_t offChannels  = MENU_NO_TEXT;
    uint16_t offIdent     = MENU_NO_TEXT;
    uint16_t offVer       = MENU_NO_TEXT;
    uint16_t offFirmware  = MENU_NO_TEXT;

    void setFixed(uint8_t idx, int32_t raw, int8_t exp)
    {
        vals[idx].kind     = (uint8_t)MenuValueKind::Fixed;
        vals[idx].raw      = raw;
        vals[idx].exponent = exp;
    }

    /** Aendert sich nur, wenn die Struktur neu gebaut werden muss. */
    static uint32_t fingerprint(const AlmemoSensorPacket &pkt, const AlmemoMenuSensorInfo &info)
    {
        uint32_t h  = 2166136261u;
        auto     fb = [&h](const void *p, size_t n) {
            const uint8_t *b = (const uint8_t *)p;
            for (size_t i = 0; i < n; i++) {
                h ^= b[i];
                h *= 16777619u;
            }
        };
        auto fs = [&fb](const char *t) { fb(t ? t : "", t ? strlen(t) : 0); };
        fs(info.source);
        fs(info.name);
        fs(info.version);
        fb(&info.typeByte, 1);
        fb(&info.i2cAddr, 1);
        fb(&info.channels, 1);
        fb(&info.channelsMax, 1);
        fb(&pkt.count, 1);
        for (uint8_t i = 0; i < pkt.count && i < ALMEMO_MAX_VALUES; i++) {
            fb(&pkt.values[i].slot, 1);
            fb(pkt.values[i].unit, 2);
        }
        return h;
    }

    uint16_t presetTextOffset() const
    {
        if (!config.lora.use_preset)
            return offCustom;
        const uint8_t p = (uint8_t)config.lora.modem_preset;
        return (p < 16 && offPreset[p] != 0) ? offPreset[p] : offCustom;
    }

    /** Label eines Messkanals nach dem Schema des Receivers: M<c-1>.<v>. */
    static void channelLabel(uint8_t slot, char *out, size_t cap)
    {
        const uint8_t ch = almemoSlotChannel(slot);
        const uint8_t vi = almemoSlotValueIndex(slot);
        if (ch == 0)
            snprintf(out, cap, "%u", vi);
        else
            snprintf(out, cap, "M%u.%u", (unsigned)(ch - 1), (unsigned)vi);
    }

    void rebuild(const AlmemoSensorPacket &pkt, const AlmemoMenuSensorInfo &info)
    {
        MenuBuilder b(s);
        char        buf[28];

        snprintf(buf, sizeof(buf), "%04X", (unsigned)(nodeDB ? (nodeDB->getNodeNum() & 0xFFFF) : 0));
        b.deviceName(buf);

        /* --- Texte, die zur Laufzeit wechseln, vorab in den Pool --- */
        offUsb    = b.str("USB versorgt");
        offCustom = b.str("Custom");
        for (uint8_t i = 0; i < STATUS_COUNT; i++)
            offStatus[i] = b.str(STATUS_TEXT[i]);

        /* Sensorinfos gehoeren in die Struktur (sie aendern sich nur mit dem
         * Fingerabdruck) — also vor finish() in den Pool. */
        char tbuf[20];
        typeText(info, tbuf, sizeof(tbuf));
        offType = b.str(tbuf);
        channelsText(info, tbuf, sizeof(tbuf));
        offChannels = b.str(tbuf);
        offIdent    = b.str(info.name && info.name[0] ? info.name : "-");
        offVer      = b.str(info.version && info.version[0] ? info.version : "-");
        offFirmware = b.str(optstr(APP_VERSION_SHORT));

        /* --- Auswahllisten: geteilt mit dem Receiver, s. AlmemoMenuSettings.h --- */
        const uint8_t enumInterval = menudef::intervalEnum(b);
        const uint8_t enumPreset   = menudef::presetEnum(b, offPreset);
        const uint8_t numInterval  = menudef::intervalNumber(b);

        /* --- Messwert-Seiten ---
         * Die Countdown-Zeile zaehlt als zusaetzliche Zeile hinter den Kanaelen
         * und wird IMMER gezeigt. Auf die letzte Seite passen dadurch nur vier
         * Kanaele, und bei vielen Messstellen kommt eine Seite dazu
         * (10 Kanaele -> 5 + 5 + Countdown = 3 Seiten). */
        const uint8_t n     = pkt.count > MAX_MEAS ? MAX_MEAS : pkt.count;
        const uint8_t rows  = (uint8_t)(n + 1); /* Kanaele + Countdown */
        const uint8_t pages = n ? (uint8_t)((rows + MENU_ROWS - 1) / MENU_ROWS) : 1;
        for (uint8_t pg = 0; pg < pages; pg++) {
            if (pages > 1)
                snprintf(buf, sizeof(buf), "MESSWERTE %u/%u", (unsigned)(pg + 1), (unsigned)pages);
            else
                snprintf(buf, sizeof(buf), "MESSWERTE");
            b.page(buf, MenuPageKind::Status);

            if (!n) {
                /* Gilt fuer beide Faelle: nichts gesteckt, und Geraet erkannt
                 * aber ohne initialisierte Messstellen. Die Sensor-Seite zeigt
                 * ohnehin, was dran ist. */
                b.item("Keine Messstellen", MENU_NO_IDX, nullptr, MenuItemKind::Hint);
                continue;
            }
            for (uint8_t r = 0; r < MENU_ROWS; r++) {
                const uint8_t i = (uint8_t)(pg * MENU_ROWS + r);
                if (i > n)
                    break;
                if (i == n) { /* letzte Zeile hinter dem letzten Kanal */
                    b.item("Aktualisierung in", VAL_NEXT, "s");
                    break;
                }
                char lbl[10], unit[4];
                channelLabel(pkt.values[i].slot, lbl, sizeof(lbl));
                unit[0] = pkt.values[i].unit[0];
                unit[1] = pkt.values[i].unit[1];
                unit[2] = '\0';
                b.item(lbl, (uint8_t)(VAL_MEAS0 + i), unit);
            }
        }

        /* --- Sensor --- */
        b.page("SENSOR", MenuPageKind::Status);
        b.item("Typ", VAL_TYPE);
        b.item("Kennung", VAL_IDENT);
        b.item("Version", VAL_VER);
        b.item("Kanäle", VAL_CHANS);
        b.item("Status", VAL_STATUS);

        /* --- Funk (RSSI/SNR fehlt: es gibt keine globale Quelle dafuer) --- */
        b.page("FUNK", MenuPageKind::Status);
        b.item("Pakete", VAL_PACKETS);
        b.item("ChUtil", VAL_CHUTIL, "%");
        b.item("AirUtilTX", VAL_AIRUTIL, "%");
        b.item("Gesendet vor", VAL_SINCE, "s");

        /* --- System --- */
        b.page("SYSTEM", MenuPageKind::Status);
        b.item("Akku", VAL_BATT, "mV");
        b.item("Firmware", VAL_FIRMWARE);

        /* --- Menue ZUERST anlegen, damit die Einstellungen-Seite seinen Index kennt.
         *     Die Karussell-Reihenfolge bleibt davon unberuehrt: nextStatusPage()
         *     ueberspringt Seiten der Art Menu. --- */
        const uint8_t pageMenu = s.pageCount;
        b.page("EINSTELLUNGEN", MenuPageKind::Menu);
        itemInterval = b.item("Sendeintervall", VAL_INTERVAL, nullptr, MenuItemKind::EditEnum, enumInterval, numInterval);
        itemPreset   = b.item("Radio-Preset", VAL_PRESET, nullptr, MenuItemKind::EditEnum, enumPreset, MENU_NO_IDX,
                              config.lora.use_preset ? 0 : MENU_ITEM_DISABLED);
        /* Nur erreichbar, solange das Display das Node-Menue zeigt -- also
         * entweder ohne aufgesteckten Receiver (dann folgenlos) oder nachdem man
         * dort "-> Node-Display" gewaehlt hat. Dies ist der Weg zurueck. */
        b.item("-> Receiver-Display", MENU_NO_IDX, nullptr, MenuItemKind::Action, MENU_ACTION_SWAP_DISPLAY);
        b.item("<- Zurück", MENU_NO_IDX, nullptr, MenuItemKind::Back);

        /* --- Einstellungen: letzte Status-Seite, langer Druck oeffnet das Menue --- */
        b.page("EINSTELLUNGEN", MenuPageKind::Status, MENU_PAGE_OPENS_MENU, pageMenu);
        b.item("Intervall", VAL_INTERVAL);
        b.item("Preset", VAL_PRESET);
        b.item("", MENU_NO_IDX, nullptr, MenuItemKind::Hint);
        b.item("", MENU_NO_IDX, nullptr, MenuItemKind::Hint);
        b.item("Lang drücken: ändern", MENU_NO_IDX, nullptr, MenuItemKind::Hint);

        b.finish(); /* ab hier NICHTS mehr in den Pool legen — der Hash ist gebildet */

        /* --- feste Werte auf die vorab abgelegten Offsets setzen --- */
        memset(vals, 0, sizeof(vals));
        setTextOff(VAL_TYPE, offType);
        setTextOff(VAL_IDENT, offIdent);
        setTextOff(VAL_VER, offVer);
        setTextOff(VAL_FIRMWARE, offFirmware);
        setTextOff(VAL_STATUS, offStatus[info.status < STATUS_COUNT ? info.status : 1]);
        setTextOff(VAL_CHANS, offChannels);
        vals[VAL_PACKETS].kind  = (uint8_t)MenuValueKind::Int;
        vals[VAL_INTERVAL].kind = (uint8_t)MenuValueKind::Duration;

        built = true;
        LOG_INFO("AlmemoMenu: Struktur neu gebaut (%u Seiten, %u Items, %u Pool, hash %08lx)", s.pageCount, s.itemCount,
                 s.poolLen, (unsigned long)s.hash);
    }

    void setTextOff(uint8_t idx, uint16_t off)
    {
        vals[idx].kind = (uint8_t)MenuValueKind::Text;
        vals[idx].text = off;
    }
};

#endif // ALMEMO_OLED
