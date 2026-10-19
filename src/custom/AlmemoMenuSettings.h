#pragma once

#include "AlmemoMenu.h"
#include "DisplayFormatters.h"
#include "NodeDB.h"

/*
 * AlmemoMenuSettings — die GETEILTE Einstellungsseite.
 *
 * Node und Receiver sollen exakt dieselben Einstellungen anbieten: dieselben
 * Intervallstufen, dieselbe Preset-Reihenfolge, dieselben Grenzen. Deshalb
 * stehen die Auswahllisten hier und nicht zweimal in den Quellen -- laufen sie
 * auseinander, passen die Optionsindizes nicht mehr zusammen, und ein
 * "Auf beide anwenden" wuerde auf dem einen Geraet etwas anderes einstellen als
 * auf dem anderen.
 *
 * Liegt bewusst NICHT in AlmemoMenu.h: das Modell bleibt frei von
 * Meshtastic-Typen. Hier sind sie erlaubt -- DisplayFormatters haengt nur an
 * NodeDB und ist in jedem Build dabei (Channels.cpp und RadioInterface.cpp
 * ziehen es ohnehin), also auch im Receiver.
 */

/* Fallback, wenn moduleConfig.telemetry.environment_update_interval 0 ist
 * (0 heisst "Default"). Gehoert zur geteilten Einstellungs-Semantik: Node und
 * Receiver muessen denselben Wert annehmen, sonst zeigen sie unterschiedliche
 * Intervalle fuer dieselbe Config. */
#ifndef ALMEMO_SENDER_DEFAULT_INTERVAL_SECS
#define ALMEMO_SENDER_DEFAULT_INTERVAL_SECS 30
#endif

namespace menudef
{

/** Sendeintervall-Stufen plus "Manuell" und Abbrechen. */
inline uint8_t intervalEnum(MenuBuilder &b)
{
    const uint8_t e = b.beginEnum();
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
    return e;
}

/** Ziffern-Eingabe fuer "Manuell": Sekunden, vierstellig. */
inline uint8_t intervalNumber(MenuBuilder &b)
{
    return b.number(10, 3600, 4);
}

/**
 * Radio-Presets, Reihenfolge nach Reichweite absteigend (aus dem Menuekonzept).
 *
 * @param offPreset Feld mit 16 Eintraegen; gefuellt wird offPreset[preset] mit
 *        dem Pool-Offset des Optionstexts, damit die Status-Seite denselben
 *        Text anzeigt, ohne ihn ein zweites Mal abzulegen.
 */
inline uint8_t presetEnum(MenuBuilder &b, uint16_t *offPreset)
{
    static const meshtastic_Config_LoRaConfig_ModemPreset PRESETS[] = {
        meshtastic_Config_LoRaConfig_ModemPreset_LONG_MODERATE, meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST,
        meshtastic_Config_LoRaConfig_ModemPreset_LONG_TURBO,    meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_SLOW,
        meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST,   meshtastic_Config_LoRaConfig_ModemPreset_SHORT_SLOW,
        meshtastic_Config_LoRaConfig_ModemPreset_SHORT_FAST,    meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO,
    };

    const uint8_t e = b.beginEnum();
    if (offPreset)
        memset(offPreset, 0, 16 * sizeof(uint16_t));
    for (auto p : PRESETS) {
        const uint16_t off = b.option(DisplayFormatters::getModemPresetDisplayName(p, false, true), (int32_t)p);
        if (offPreset && (uint8_t)p < 16)
            offPreset[(uint8_t)p] = off;
    }
    b.option("<- Abbrechen", MENU_OPT_CANCEL);
    return e;
}

} // namespace menudef
