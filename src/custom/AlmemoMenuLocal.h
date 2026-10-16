#pragma once

#include "configuration.h"

#if defined(ALMEMO_OLED)

#include "AlmemoCommon.h"
#include "AlmemoMenu.h"
#include "DisplayFormatters.h"
#include "NodeDB.h"
#include "PowerStatus.h"
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

/** Was der Sender ueber seine aktuelle Quelle mitteilt. */
struct AlmemoMenuSensorInfo {
    const char *source     = "NONE";
    const char *connection = "";
    const char *name       = "";
    const char *version    = "";
    uint8_t     channels   = 0;
    uint8_t     status     = 1; /* Index in STATUS_TEXT */
};

class AlmemoMenuLocal
{
  public:
    /* Feste Werteplaetze — der Builder verdrahtet die Items darauf. */
    enum : uint8_t {
        VAL_MEAS0 = 0, /* 0..9: Messwerte */
        VAL_CONN  = 10,
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
        VAL_COUNT
    };
    static constexpr uint8_t MAX_MEAS = 10;

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

        vals[VAL_PACKETS].raw = (int32_t)sendCount;
        vals[VAL_INTERVAL].raw = (int32_t)intervalSecs;
        lastSendMs             = millis();
        poll();
    }

    /** Sensor weg: Werte verwerfen, Struktur auf "Kein Sensor" stellen. */
    void clear()
    {
        AlmemoSensorPacket empty = {};
        AlmemoMenuSensorInfo info;
        rebuild(empty, info);
        lastFingerprint = fingerprint(empty, info);
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

        vals[VAL_SINCE].raw  = lastSendMs ? (int32_t)((millis() - lastSendMs) / 1000) : 0;
        vals[VAL_SINCE].kind = lastSendMs ? (uint8_t)MenuValueKind::Int : (uint8_t)MenuValueKind::Invalid;

        /* Akku: bei USB-Versorgung Text statt Spannung (Einheit wird dann nicht angehaengt). */
        if (powerStatus && powerStatus->getHasUSB()) {
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
            vals[VAL_INTERVAL].raw = e.raw;
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

    uint8_t  itemInterval = MENU_NO_IDX;
    uint8_t  itemPreset   = MENU_NO_IDX;
    uint16_t offUsb       = MENU_NO_TEXT;
    uint16_t offPreset[16]{};  /* Pool-Offset je Preset-Enumwert */
    uint16_t offCustom    = MENU_NO_TEXT;
    uint16_t offStatus[STATUS_COUNT]{};
    uint16_t offConn      = MENU_NO_TEXT;
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
        fs(info.connection);
        fs(info.name);
        fs(info.version);
        fb(&info.channels, 1);
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
        offConn     = b.str(info.connection && info.connection[0] ? info.connection : (info.source ? info.source : "-"));
        offIdent    = b.str(info.name && info.name[0] ? info.name : "-");
        offVer      = b.str(info.version && info.version[0] ? info.version : "-");
        offFirmware = b.str(optstr(APP_VERSION_SHORT));

        /* --- Auswahllisten --- */
        const uint8_t enumInterval = b.beginEnum();
        b.option("10s", 10);
        b.option("30s", 30);
        b.option("1min", 60);
        b.option("2min", 120);
        b.option("5min", 300);
        b.option("10min", 600);
        b.option("15min", 900);
        b.option("30min", 1800);
        b.option("60min", 3600);
        b.option("Manuell", MENU_OPT_MANUAL);
        b.option("<- Abbrechen", MENU_OPT_CANCEL);

        /* Reihenfolge nach Reichweite absteigend, wie im Menuekonzept. */
        static const meshtastic_Config_LoRaConfig_ModemPreset PRESETS[] = {
            meshtastic_Config_LoRaConfig_ModemPreset_LONG_MODERATE, meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
            meshtastic_Config_LoRaConfig_ModemPreset_LONG_TURBO,    meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_SLOW,
            meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST,   meshtastic_Config_LoRaConfig_ModemPreset_SHORT_SLOW,
            meshtastic_Config_LoRaConfig_ModemPreset_SHORT_FAST,    meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO,
        };
        const uint8_t enumPreset = b.beginEnum();
        memset(offPreset, 0, sizeof(offPreset));
        for (auto p : PRESETS) {
            /* Der Optionstext ist derselbe, den die Einstellungs-Seite anzeigt —
             * einmal ablegen, Offset fuer den Wert merken. */
            const uint16_t off = b.option(DisplayFormatters::getModemPresetDisplayName(p, false, true), (int32_t)p);
            if ((uint8_t)p < 16)
                offPreset[(uint8_t)p] = off;
        }
        b.option("<- Abbrechen", MENU_OPT_CANCEL);

        const uint8_t numInterval = b.number(10, 3600, 4);

        /* --- Messwert-Seiten: 5 Kanaele pro Seite --- */
        const uint8_t n     = pkt.count > MAX_MEAS ? MAX_MEAS : pkt.count;
        const uint8_t pages = n ? (uint8_t)((n + MENU_ROWS - 1) / MENU_ROWS) : 1;
        for (uint8_t pg = 0; pg < pages; pg++) {
            if (pages > 1)
                snprintf(buf, sizeof(buf), "MESSWERTE %u/%u", (unsigned)(pg + 1), (unsigned)pages);
            else
                snprintf(buf, sizeof(buf), "MESSWERTE");
            b.page(buf, MenuPageKind::Status);

            if (!n) {
                b.item("Kein Sensor", MENU_NO_IDX, nullptr, MenuItemKind::Hint);
                continue;
            }
            for (uint8_t r = 0; r < MENU_ROWS; r++) {
                const uint8_t i = (uint8_t)(pg * MENU_ROWS + r);
                if (i >= n)
                    break;
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
        b.item("Anschluss", VAL_CONN);
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
        setTextOff(VAL_CONN, offConn);
        setTextOff(VAL_IDENT, offIdent);
        setTextOff(VAL_VER, offVer);
        setTextOff(VAL_FIRMWARE, offFirmware);
        setTextOff(VAL_STATUS, offStatus[info.status < STATUS_COUNT ? info.status : 1]);
        vals[VAL_CHANS].kind    = (uint8_t)MenuValueKind::Int;
        vals[VAL_CHANS].raw     = info.channels;
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
