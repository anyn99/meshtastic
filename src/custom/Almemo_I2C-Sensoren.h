#pragma once

#include <Wire.h>
#include <stdint.h>

/**
 * AlmemoI2CSensor — Treiber für ALMEMO-Sensoren am I2C-Bus.
 *
 * Speicherlayout an 0x50 (EEPROM-ähnlich):
 *   0x00..0x07   Sensorname (ASCII, 8 Byte)
 *   0x08..0x43   Sensor-1 Infoblock (60 Byte)
 *   0x44..0x7F   Sensor-2 Infoblock
 *   0x80..0xBB   Sensor-3 Infoblock
 *   0xBC..0xF7   Sensor-4 Infoblock
 *   0xF8..0xFF   Versionsstring (ASCII, 8 Byte)
 *
 * Pro Sensor-Slot (Basis = 0x08 + slot * 0x3C):
 *   +0x00  Typ          (0x37=digital, 0x09=analog NTC, 0xFF=kein Sensor)
 *   +0x01  Exponent     (oberes Nibble bei Digital: Bit3=Vorzeichen, Bit2-0=|Exp|)
 *   +0x1E  Einheit      (ASCII, 2 Byte)
 *   +0x20  Kommentar    (ASCII, 10 Byte)
 *
 * Messwerte über I2C-Adresse 0x40, Register = Slot-Index 0..3:
 *   write 0x40 reg=slot
 *   read  0x40            → 4 Byte [0x00, status, MSB, LSB]
 *     status = 0x40 → gültig
 *     status = 0x80 → kein/ungültiger Wert
 *     value (int16 BE) wird mit 10^exponent skaliert
 */
class AlmemoI2CSensor
{
  public:
    static constexpr uint8_t EEPROM_ADDR = 0x50;
    static constexpr uint8_t VALUE_ADDR  = 0x40;
    static constexpr uint8_t MAX_SLOTS   = 4;
    static constexpr uint8_t SLOT_SIZE   = 0x3C; /* 60 */

    enum SensorType : uint8_t {
        TYPE_NONE    = 0xFF,
        TYPE_DIGITAL = 0x37,
        TYPE_ANALOG  = 0x09,
    };

    struct SlotInfo {
        bool    present;     /* type != 0xFF */
        uint8_t type;
        int8_t  exponent;    /* dekodiert aus oberem Nibble, Vorzeichen-Magnitude */
        char    unit[3];     /* 2 ASCII + NUL */
    };

    /**
     * Reines Adress-Probing: ein einzelner Write-Versuch auf 0x50.
     * @return true wenn ACK.
     */
    static bool probe(TwoWire &wire);

    /**
     * Probe + Metadaten einlesen (Name, Version, alle 4 Slot-Infos).
     * Loggt erkannte Sensoren als LOG_INFO.
     * @return true wenn 0x50 erreichbar.
     */
    bool begin(TwoWire &wire);

    /**
     * Misswert von Slot N lesen. Skaliert mit dekodiertem Exponent.
     * @return true bei gültigem Wert (status=0x40); false sonst.
     */
    bool readValue(uint8_t slot, float &out);

    const SlotInfo &slot(uint8_t i) const { return slots[i]; }
    const char     *deviceName() const { return name; }
    const char     *deviceVersion() const { return version; }
    uint8_t         numPresent() const;

  private:
    TwoWire *wire = nullptr;
    SlotInfo slots[MAX_SLOTS]{};
    char     name[9]{};
    char     version[9]{};

    bool readMem(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
    void parseSlot(uint8_t i, const uint8_t *block);
};
