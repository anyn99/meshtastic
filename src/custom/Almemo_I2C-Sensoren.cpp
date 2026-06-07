#include "Almemo_I2C-Sensoren.h"

#include "configuration.h"

namespace
{
/* 10^e Lookup für Exponenten -4..+4 (digitale ALMEMO-Sensoren). */
float pow10f(int8_t e)
{
    static const float tbl[] = {1.0f, 10.0f, 100.0f, 1000.0f, 10000.0f};
    if (e >= 0)
        return (e <= 4) ? tbl[e] : 1.0f;
    return (e >= -4) ? (1.0f / tbl[-e]) : 1.0f;
}
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

void AlmemoI2CSensor::parseSlot(uint8_t i, const uint8_t *block)
{
    SlotInfo &s = slots[i];
    s.type     = block[0];
    s.present  = (s.type != TYPE_NONE);

    /* Exponent aus oberem Nibble (digital):
     *   Bit3 = Vorzeichen (1 → negativ), Bit2-0 = Magnitude 0..4. */
    uint8_t hi = (block[1] >> 4) & 0x0F;
    uint8_t mag = hi & 0x07;
    bool    neg = (hi & 0x08) != 0;
    s.exponent  = neg ? -(int8_t)mag : (int8_t)mag;

    s.unit[0] = (char)block[0x1E];
    s.unit[1] = (char)block[0x1F];
    s.unit[2] = '\0';
}

bool AlmemoI2CSensor::begin(TwoWire &w)
{
    wire = &w;

    if (!probe(*wire)) {
        LOG_DEBUG("AlmemoI2C: no ACK @0x%02x", EEPROM_ADDR);
        return false;
    }

    uint8_t buf[SLOT_SIZE];

    if (readMem(EEPROM_ADDR, 0x00, (uint8_t *)name, 8))
        name[8] = '\0';
    if (readMem(EEPROM_ADDR, 0xF8, (uint8_t *)version, 8))
        version[8] = '\0';

    static constexpr uint8_t SLOT_BASE[MAX_SLOTS] = {0x08, 0x44, 0x80, 0xBC};

    for (uint8_t i = 0; i < MAX_SLOTS; i++) {
        if (!readMem(EEPROM_ADDR, SLOT_BASE[i], buf, SLOT_SIZE)) {
            slots[i] = SlotInfo{};
            continue;
        }
        parseSlot(i, buf);
        if (slots[i].present) {
            LOG_INFO("AlmemoI2C: slot %u type=0x%02x exp=%d unit='%s'",
                     i, slots[i].type, slots[i].exponent, slots[i].unit);
        }
    }

    LOG_INFO("AlmemoI2C: name='%s' ver='%s' present=%u", name, version, numPresent());
    return true;
}

AlmemoI2CSensor::ReadResult AlmemoI2CSensor::readValue(uint8_t slot, float &out)
{
    if (!wire || slot >= MAX_SLOTS || !slots[slot].present)
        return ReadResult::NotConnected;

    uint8_t buf[4];
    /* Kein ACK auf 0x40 → Sensor ist vom Bus abgesteckt. */
    if (!readMem(VALUE_ADDR, slot, buf, 4))
        return ReadResult::NotConnected;

    /* buf[0] ist üblicherweise 0x00, buf[1] = Status:
     *   0x40 → gültig, 0x80 → kein/ungültiger Wert.
     * Gerät antwortet, liefert aber keinen gültigen Wert → echter Lesefehler. */
    if (buf[1] != 0x40)
        return ReadResult::BadValue;

    int16_t raw = (int16_t)((uint16_t)buf[2] << 8 | buf[3]);
    out = (float)raw * pow10f(slots[slot].exponent);
    return ReadResult::Ok;
}

uint8_t AlmemoI2CSensor::numPresent() const
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < MAX_SLOTS; i++)
        if (slots[i].present)
            n++;
    return n;
}
