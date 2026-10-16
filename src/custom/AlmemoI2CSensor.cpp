#include "AlmemoI2CSensor.h"

#include "configuration.h"

#include <string.h>

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

/* ALMEMO fuellt die EEPROM-Textfelder mit Leerzeichen auf, und zwar je Feld
 * unterschiedlich: der Name hinten ("FHAD46  "), die Version vorne ("   6.66").
 * Deshalb beide Enden trimmen -- und zwar hier im Treiber, damit jeder
 * Verbraucher einen sauberen String bekommt und nicht jede Anzeige erneut
 * trimmen muss. */
void trimSpaces(char *s)
{
    char *a = s;
    while (*a == ' ')
        a++;
    if (a != s)
        memmove(s, a, strlen(a) + 1);
    for (size_t n = strlen(s); n > 0 && s[n - 1] == ' '; n--)
        s[n - 1] = '\0';
}
} // namespace

bool AlmemoI2CSensor::probe(TwoWire &wire)
{
    /* ALMEMO-Master verlangt teils eine Wake-up-Sequenz vor dem ersten ACK:
     * bis zu 3 Versuche.
     *
     * Form adressonly, wie beim ALMEMO-Logger ("write to 0x50" + ACK, den er
     * regelmaessig zur Steckerkennung macht).
     *
     * Hier stand kurzzeitig ein Registerbyte, weil vermutet wurde, bei MAXCNT=0
     * folge der STOP zu dicht auf das ACK und der Slave bleibe deshalb in
     * ST_WRITE_RX haengen. I2C-Mitschnitte haben das WIDERLEGT: bei 100 kHz
     * liegt die SCL-Fallflanke +5,0 us nach dem ACK und der STOP bei +25 us --
     * die Transaktion ist formal einwandfrei (Logger bei 49 kHz: +8..9 us bzw.
     * +31 us, also nur die halbe Geschwindigkeit, nicht eine andere Form).
     *
     * Die Ursache war der BUSTAKT: der Bitbang-Slave des Receivers antwortet
     * bei ~49 kHz zu 100 %, bei 100 kHz nur zu 6 %, weil seine ISR 3-6 us
     * braucht und die SCL-Halbperiode dort nur 5,0 us betraegt. Verschaerft
     * durch sht.begin(), das bei jedem erfolglosen Erkennungslauf Wire.begin()
     * rief und den Takt auf K100 zurueckriss. Beides behoben, s.
     * AlmemoI2CClock.h -- der Probe darf also wieder die schlanke Form haben.
     */
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
    typeByte  = ALMEMO_TYPE_NONE;

    if (!probe(*wire)) {
        //LOG_DEBUG("AlmemoI2C: no ACK @0x%02x", EEPROM_ADDR);
        return false;
    }

    uint8_t buf[SLOT_SIZE];

    if (readMem(EEPROM_ADDR, 0x00, (uint8_t *)name, 8)) {
        name[8] = '\0';
        trimSpaces(name);
    }
    if (readMem(EEPROM_ADDR, 0xF8, (uint8_t *)version, 8)) {
        version[8] = '\0';
        trimSpaces(version);
    }

    /* Die Generation steckt in der VERSIONSNUMMER (immer an 0xF8 lesbar, auch
     * ohne eine einzige initialisierte Messstelle): 6.xx = D6, 7.xx = D7. Ein
     * Slot-Byte taugt dafuer nicht, weil es erst existiert, wenn ein Slot
     * belegt ist.
     *
     * Ein D7 ackt auf 0x50 ebenfalls, hat dort aber kein EEPROM-Slot-Layout.
     * Hier gleich abweisen, damit probeSensors() auf den D7-Pfad durchfallen
     * kann -- auch in Builds ohne ALMEMO_SENSOR_D7, wo der D7-Probe nicht
     * vorlaeuft -- und damit die vier Seitenlesungen unten entfallen. */
    if (version[0] == '7' && version[1] == '.')
        return false;
    if (version[0] == '6' && version[1] == '.')
        typeByte = ALMEMO_TYPE_D6;

    /* Version und Slot-Byte beschreiben zwei Achsen: die Version die
     * Generation, das Slot-Byte die Sensorart (0x09 analog / 0x37 digital).
     * Deshalb gilt die Version nur als Vorbelegung und der erste belegte Slot
     * praezisiert sie -- so bleibt ANALOG erhalten, und ohne belegten Slot
     * bleibt es bei der Generation. */
    bool typeFromSlot = false;

    static constexpr uint8_t SLOT_BASE[MAX_SLOTS] = {0x08, 0x44, 0x80, 0xBC};

    /* Für jeden EEPROM-Slot mit bekanntem Sensortyp einen AlmemoSlot anlegen —
     * kompakte Liste, leere/fremde Slots werden übersprungen. */
    for (uint8_t hw = 0; hw < MAX_SLOTS; hw++) {
        if (!readMem(EEPROM_ADDR, SLOT_BASE[hw], buf, SLOT_SIZE))
            continue;
        AlmemoSlot s;
        if (parseSlot(hw, buf, s)) {
            /* Erster belegter Slot praezisiert die Vorbelegung (s. oben). */
            if (!typeFromSlot) {
                typeFromSlot = true;
                typeByte     = buf[0];
            }
            slots[slotCount++] = s;
            LOG_INFO("AlmemoI2C: slot %u (hw %u) exp=%d unit='%s'", slotCount - 1, hw, s.exponent, s.unit);
        }
    }

    /* Bewusst JEDES Mal: die Zeile soll beim An- und Abstecken erscheinen.
     *
     * Hier stand kurzzeitig ein Gate auf Aenderungen, weil begin() bei 0 Slots
     * scheiterte und deshalb im 500-ms-Probe-Intervall endlos durchlief. Seit
     * Nullslot-Geraete akzeptiert werden, wird die Quelle gesetzt und
     * probeSensors() hoert auf -- der Grund ist also weg.
     *
     * Bleibt eine laute Stelle: bei SRC_ALMEMO_MULTI ruft refreshMux() begin()
     * in jedem Sendezyklus pro Kanal. Das war aber auch vorher so. */
    LOG_INFO("AlmemoI2C: name='%s' ver='%s' slots=%u", name, version, slotCount);
    /* Das Geraet gilt als erkannt, sobald 0x50 ackt -- AUCH ohne einen
     * einzigen belegten Slot. Ein Fuehler ohne initialisierte Messstellen (etwa
     * unser Receiver, bevor sein Sensor eingerichtet ist) ist dann eben ein
     * Geraet ohne Messwerte: Kennung und Version stehen, Kanaele 0, keine
     * Werte. Der Leseweg meldet das als ReadStatus::Empty. */
    return true;
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
