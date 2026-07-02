#include "AlmemoI2CSensor.h"

#include "configuration.h"

namespace
{
/* Bekannte ALMEMO-Sensortypen (Byte an Offset 0 des EEPROM-Slots). Nur diese
 * gelten als belegter Slot; alles andere (leerer Slot 0xFF, D7-Kennung 0x7B,
 * Busmüll) legt keinen Slot an. Iterierbar → neuen Sensortyp einfach als weitere
 * Zeile ergänzen. */
constexpr uint8_t KNOWN_SENSOR_TYPES[] = {
    0x37, // digital
    0x09, // analog (NTC)
};
} // namespace

bool AlmemoI2CSensor::probe(TwoWire &wire)
{
    /* ALMEMO-Master verlangt teils eine Wake-up-Sequenz vor dem ersten ACK:
     * bis zu 3 Adress-Writes ohne Payload. */
    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        wire.beginTransmission(EEPROM_ADDR);
        if (wire.endTransmission() == 0)
            return true;
    }
    return false;
}

bool AlmemoI2CSensor::readMem(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    /* Write-then-read als getrennte Transaktionen (kein repeated start),
     * passend zum beobachteten ALMEMO-Bus-Verhalten. */
    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        wire->beginTransmission(addr);
        wire->write(reg);
        if (wire->endTransmission() == 0)
            break;
        if (attempt == 2)
            return false;
    }

    uint8_t got = wire->requestFrom((int)addr, (int)len);
    if (got != len)
        return false;
    for (uint8_t i = 0; i < len; i++)
        buf[i] = wire->read();
    return true;
}

bool AlmemoI2CSensor::parseSlot(uint8_t hwSlot, const uint8_t *block, AlmemoSlot &out)
{
    /* Typ-Byte gegen die bekannten Sensortypen prüfen. 0xFF = leerer Slot. */
    uint8_t type  = block[0];
    bool    known = false;
    for (uint8_t kt : KNOWN_SENSOR_TYPES)
        if (type == kt) {
            known = true;
            break;
        }
    if (!known) {
        if (type != 0xFF)
            LOG_DEBUG("AlmemoI2C: Unknown Sensor Type 0x%02X", type);
        return false;
    }

    out.valueIndex = hwSlot; // EEPROM-Slot-Index → Paket-valueIndex

    /* Exponent aus oberem Nibble (digital):
     *   Bit3 = Vorzeichen (1 → negativ), Bit2-0 = Magnitude 0..4. */
    uint8_t hi  = (block[1] >> 4) & 0x0F;
    uint8_t mag = hi & 0x07;
    bool    neg = (hi & 0x08) != 0;
    out.exponent = neg ? -(int8_t)mag : (int8_t)mag;

    out.unit[0] = (char)block[0x1E];
    out.unit[1] = (char)block[0x1F];
    out.unit[2] = '\0';
    return true;
}

bool AlmemoI2CSensor::begin(TwoWire &w)
{
    wire      = &w;
    slotCount = 0;

    if (!probe(*wire)) {
        //LOG_DEBUG("AlmemoI2C: no ACK @0x%02x", EEPROM_ADDR);
        return false;
    }

    uint8_t buf[SLOT_SIZE];

    if (readMem(EEPROM_ADDR, 0x00, (uint8_t *)name, 8))
        name[8] = '\0';
    if (readMem(EEPROM_ADDR, 0xF8, (uint8_t *)version, 8))
        version[8] = '\0';

    static constexpr uint8_t SLOT_BASE[MAX_SLOTS] = {0x08, 0x44, 0x80, 0xBC};

    /* Für jeden EEPROM-Slot mit bekanntem Sensortyp einen AlmemoSlot anlegen —
     * kompakte Liste, leere/fremde Slots werden übersprungen. */
    for (uint8_t hw = 0; hw < MAX_SLOTS; hw++) {
        if (!readMem(EEPROM_ADDR, SLOT_BASE[hw], buf, SLOT_SIZE))
            continue;
        AlmemoSlot s;
        if (parseSlot(hw, buf, s)) {
            slots[slotCount++] = s;
            LOG_INFO("AlmemoI2C: slot %u (hw %u) exp=%d unit='%s'", slotCount - 1, hw, s.exponent, s.unit);
        }
    }

    LOG_INFO("AlmemoI2C: name='%s' ver='%s' slots=%u", name, version, slotCount);
    /* 0x50 ackt zwar, aber ohne einen einzigen bekannten Sensor-Typ ist das kein
     * ALMEMO-EEPROM-Gerät (z. B. ein D7 oder Fremdgerät) → als „nicht da" melden. */
    return slotCount > 0;
}

AlmemoI2CSensor::ReadResult AlmemoI2CSensor::readRaw(uint8_t i, int16_t &raw)
{
    if (!wire || i >= slotCount)
        return ReadResult::NotConnected;

    uint8_t buf[4];
    /* Wertregister = EEPROM-Slot-Index des Slots. Kein ACK auf 0x40 → abgesteckt. */
    if (!readMem(VALUE_ADDR, slots[i].valueIndex, buf, 4))
        return ReadResult::NotConnected;

    /* buf[0] ist üblicherweise 0x00, buf[1] = Status:
     *   0x40 → gültig, 0x80 → kein/ungültiger Wert.
     * Gerät antwortet, liefert aber keinen gültigen Wert → echter Lesefehler. */
    if (buf[1] != 0x40)
        return ReadResult::BadValue;

    raw = (int16_t)((uint16_t)buf[2] << 8 | buf[3]);
    return ReadResult::Ok;
}

AlmemoI2CSensor::ReadResult AlmemoI2CSensor::readValue(uint8_t i, float &out)
{
    int16_t raw;
    ReadResult rr = readRaw(i, raw);
    if (rr == ReadResult::Ok)
        out = almemoScaleFromRaw(raw, slots[i].exponent);
    return rr;
}
