#pragma once

#include "AlmemoI2CClock.h"
#include "AlmemoMenu.h"
#include "AlmemoMenuWire.h"
#include "configuration.h"

#include <Wire.h>

/*
 * AlmemoMenuRemote — Menuequelle des aufgesteckten Receivers.
 *
 * Gegenstueck zu AlmemoMenuLocal: liefert eine MenuStruct und einen Wertesatz,
 * nur eben ueber I2C geholt statt selbst gebaut. Renderer und Controller sehen
 * keinen Unterschied -- genau dafuer ist das Modell POD und ohne
 * Funktionszeiger.
 *
 * Ablauf von poll():
 *   1. Kopf lesen (20 B) und das Magic pruefen
 *   2. Hash mit dem letzten vergleichen -- gleich? dann nur die Werte holen
 *   3. Bei neuem Hash die Struktur in Bloecken holen und NACHRECHNEN
 *   4. Werte holen
 *
 * Schritt 3 ist nicht optional: der Receiver baut seine Struktur im laufenden
 * Betrieb neu (neue Messstelle), ohne dafuer Interrupts zu sperren -- er wuerde
 * sonst den Logger nicht hoeren, der 0x50 zur Steckerkennung pollt. Ein Abruf
 * kann also mitten in einen Rebuild fallen und eine zerrissene Struktur
 * liefern. Das faengt allein die Hash-Pruefung ab; sie macht den Vorgang
 * selbstheilend, weil der naechste Versuch die stabile Fassung bekommt.
 *
 * Kosten: die Struktur sind ~1,5 kB in 24 Bloecken, rund 295 ms am 48,8-kHz-Bus
 * -- und das BLOCKIEREND, alle OSThreads haengen kooperativ in derselben
 * Main-Loop. Deshalb wird sie nur bei geaendertem Hash geholt. Die Werte sind
 * mit ~320 B deutlich billiger (~60 ms) und laufen im Anzeige-Intervall.
 */

class AlmemoMenuRemote
{
  public:
    /** Struktur, oder nullptr solange keine gueltige geholt wurde. */
    const MenuStruct *structure() const { return valid ? &s : nullptr; }
    const MenuValue  *values() const { return vals; }
    uint32_t          hash() const { return valid ? s.hash : 0; }

    /** Alles vergessen (Receiver abgezogen). */
    void clear()
    {
        valid    = false;
        lastHash = 0;
    }

    /**
     * Kopf pollen, bei Bedarf Struktur und dann Werte holen.
     * @return true wenn sich etwas Anzeigbares geaendert hat.
     */
    bool poll()
    {
        almemoI2cClockGuard();

        MenuWireHeader h{};
        if (!readWindow(ALMEMO_MENU_OFF_HEADER, (uint8_t *)&h, sizeof(h)) || h.magic != ALMEMO_MENU_MAGIC) {
            const bool had = valid;
            clear();
            return had;
        }

        /* Verträglichkeit: die Struktur kommt ROH, also muessen die Kapazitaeten
         * beider Seiten uebereinstimmen. Siehe AlmemoMenuWire.h. */
        if (h.modelVersion != ALMEMO_MENU_MODEL_VERSION || h.structSize != (uint16_t)sizeof(MenuStruct) ||
            h.pageCount > MENU_MAX_PAGES || h.itemCount > MENU_MAX_ITEMS || h.valueCount > MENU_MAX_VALUES ||
            h.enumCount > MENU_MAX_ENUMS || h.optionCount > MENU_MAX_OPTIONS ||
            h.numberCount > MENU_MAX_NUMBERS || h.poolLen > MENU_POOL) {
            const bool had = valid;
            clear();
            return had;
        }

        bool changed = false;

        if (!valid || h.hash != lastHash) {
            if (!fetchStructure(h))
                return false; /* naechster Versuch -- lastHash bleibt unberuehrt */
            lastHash = h.hash;
            valid    = true;
            changed  = true;
        }

        /* Werte: nur die belegten Eintraege, der Rest bleibt wie er ist. */
        const uint16_t vbytes = (uint16_t)(s.valueCount * sizeof(MenuValue));
        if (vbytes) {
            MenuValue fresh[MENU_MAX_VALUES];
            memcpy(fresh, vals, sizeof(fresh));
            if (readWindow(ALMEMO_MENU_OFF_VALUES, (uint8_t *)fresh, vbytes) &&
                memcmp(fresh, vals, vbytes) != 0) {
                memcpy(vals, fresh, vbytes);
                changed = true;
            }
        }
        return changed;
    }

  private:
    /**
     * Struktur holen und gegen ihren eigenen Hash pruefen.
     *
     * Verglichen wird gegen den Hash AUS DER GEHOLTEN Struktur und zusaetzlich
     * gegen den aus dem Kopf: so fallen sowohl ein zerrissener Abruf als auch
     * ein Rebuild zwischen Kopf- und Strukturlesung auf.
     */
    bool fetchStructure(const MenuWireHeader &h)
    {
        if (!readWindow(ALMEMO_MENU_OFF_STRUCT, (uint8_t *)&tmp, (uint16_t)sizeof(tmp)))
            return false;

        const uint32_t calc = menuHash(tmp);
        if (calc != tmp.hash || tmp.hash != h.hash) {
            LOG_WARN("AlmemoMenuRemote: Struktur verworfen (Kopf %08lx, Struktur %08lx, gerechnet %08lx)",
                     (unsigned long)h.hash, (unsigned long)tmp.hash, (unsigned long)calc);
            return false;
        }

        memcpy(&s, &tmp, sizeof(s));
        LOG_INFO("AlmemoMenuRemote: Struktur geholt (%u Seiten, %u Items, %u Werte, %u Pool, hash %08lx)",
                 s.pageCount, s.itemCount, s.valueCount, s.poolLen, (unsigned long)s.hash);
        return true;
    }

    /** Fenster in ALMEMO_MENU_CHUNK-Bloecken lesen, Zeiger je Block neu setzen. */
    static bool readWindow(uint16_t off, uint8_t *dst, uint16_t len)
    {
        while (len) {
            const uint8_t n = len > ALMEMO_MENU_CHUNK ? (uint8_t)ALMEMO_MENU_CHUNK : (uint8_t)len;

            /* Der Slave zaehlt seinen Zeiger selbst weiter; wir setzen ihn
             * trotzdem pro Block neu. Kostet 3 Byte und macht den Abruf
             * selbstsynchronisierend, falls eine Transaktion verloren geht. */
            Wire.beginTransmission(ALMEMO_MENU_I2C_ADDR);
            Wire.write((uint8_t)(off >> 8));
            Wire.write((uint8_t)(off & 0xFF));
            if (Wire.endTransmission() != 0)
                return false;
            if (Wire.requestFrom((int)ALMEMO_MENU_I2C_ADDR, (int)n) != n)
                return false;
            for (uint8_t i = 0; i < n; i++)
                dst[i] = (uint8_t)Wire.read();

            dst += n;
            off = (uint16_t)(off + n);
            len = (uint16_t)(len - n);
        }
        return true;
    }

    MenuStruct s{};
    MenuValue  vals[MENU_MAX_VALUES]{};

    /* Zwischenpuffer fuer den Abruf, BEWUSST als Member und nicht auf dem Stack:
     * eine MenuStruct ist ~1,5 kB, und die Main-Loop-Task hat nur 4 kB
     * (LOOP_STACK_SZ im Adafruit-nRF52-Core). Zusammen mit dem Wertepuffer im
     * Aufrufer und der stackhungrigen LOG_*-Formatierung waere das die Haelfte
     * des Stacks in einem Aufrufpfad -- ein Ueberlauf, den kein Build findet.
     * Erst nach bestandener Hash-Pruefung wird nach s kopiert. */
    MenuStruct tmp{};

    uint32_t lastHash = 0;
    bool     valid    = false;
};
