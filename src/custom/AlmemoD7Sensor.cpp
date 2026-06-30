#include "AlmemoD7Sensor.h"

#include "configuration.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace
{
/* UART-Verkehr lesbar nach LOG_DEBUG ausgeben: druckbares ASCII direkt, ETX als
 * <ETX>, alles andere als \xXX. dir = "TX" (unsere Eingaben) / "RX" (Sensor). */
void d7LogBytes(const char *dir, const uint8_t *data, size_t len)
{
    char   out[160];
    size_t j = 0;
    bool   truncated = false;
    for (size_t i = 0; i < len; i++) {
        if (j > sizeof(out) - 6) { // Platz fuer "\xXX" + NUL
            truncated = true;
            break;
        }
        uint8_t c = data[i];
        if (c == 0x03)
            j += snprintf(out + j, sizeof(out) - j, "<ETX>");
        else if (c >= 0x20 && c < 0x7F)
            out[j++] = (char)c;
        else
            j += snprintf(out + j, sizeof(out) - j, "\\x%02X", c);
    }
    out[j] = '\0';
    LOG_DEBUG("AlmemoD7 %s[%u]: %s%s", dir, (unsigned)len, out, truncated ? " …" : "");
}
} // namespace

bool AlmemoD7Sensor::probe(TwoWire &wire)
{
    /* Wake-up: der ALMEMO verlangt teils mehrere Adress-Writes, bevor er ackt
     * (bis zu 3 Versuche, passend zum mitgeschnittenen Handshake). */
    bool acked = false;
    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        wire.beginTransmission(I2C_ADDR);
        if (wire.endTransmission() == 0) {
            acked = true;
            break;
        }
    }
    if (!acked)
        return false;

    /* Typ-Byte aus Register 0x08 lesen (Write reg, dann Read). */
    wire.beginTransmission(I2C_ADDR);
    wire.write(REG_TYPE);
    if (wire.endTransmission() != 0)
        return false;
    if (wire.requestFrom((int)I2C_ADDR, 1) != 1)
        return false;

    return wire.read() == TYPE_D7;
}

void AlmemoD7Sensor::beginUart(Stream &serial)
{
    uart = &serial;
    runSetup();
}

void AlmemoD7Sensor::runSetup()
{
    if (!uart)
        return;

    /* Preamble exakt aus dem Logger-Mitschnitt: "G-01" + 24 Binärbytes + "G-01".
     * (Geräte-Adressierung/Sync; Bedeutung der Binärbytes nicht weiter ausgewertet,
     * wird 1:1 wie vom Original-Logger gesendet.) */
    static const uint8_t preamble[] = {
        'G', '-', '0', '1',
        0x80, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        'G', '-', '0', '1',
    };
    uart->write(preamble, sizeof(preamble));
    d7LogBytes("TX", preamble, sizeof(preamble));
    drainUntilEtx(150);

    /* Setup-Kommandos in exakter Reihenfolge (P65 und f4 kommen doppelt vor).
     * Nach jedem Kommando die Antwort (…\x03) leeren, um synchron zu bleiben. */
    static const char *const cmds[] = {
        "t0", "f4", "k0", "N2", "P15", "P64", "P65", "f1", "P65", "f4", "P00",
    };
    for (const char *c : cmds) {
        uart->write((const uint8_t *)c, strlen(c));
        d7LogBytes("TX", (const uint8_t *)c, strlen(c));
        drainUntilEtx(200);
    }

    LOG_INFO("AlmemoD7: UART setup done @%lu baud", (unsigned long)UART_BAUD);
}

bool AlmemoD7Sensor::drainUntilEtx(uint32_t timeoutMs)
{
    if (!uart)
        return false;
    uint8_t  buf[120]; // nur fuers Debug-Log; weiter gelesen wird auch ohne Platz
    size_t   n     = 0;
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        while (uart->available()) {
            uint8_t c = uart->read();
            if (n < sizeof(buf))
                buf[n++] = c;
            start = millis(); // solange Bytes fließen, Timeout nachladen
            if (c == 0x03) {
                d7LogBytes("RX", buf, n);
                return true;
            }
        }
    }
    if (n) // Timeout: zeigen, was (unvollständig) ankam
        d7LogBytes("RX", buf, n);
    return false;
}

bool AlmemoD7Sensor::readValue(float &out)
{
    if (!uart)
        return false;

    while (uart->available()) // alte/Restbytes verwerfen
        uart->read();

    uart->write('='); // Abfrage
    const uint8_t q = '=';
    d7LogBytes("TX", &q, 1);

    char     buf[32];
    size_t   n     = 0;
    uint32_t start = millis();
    while (millis() - start < 100) {
        while (uart->available()) {
            char ch = uart->read();
            if (ch == 0x03) { // ETX → Antwort komplett
                buf[n] = '\0';
                d7LogBytes("RX", (const uint8_t *)buf, n);
                return parseValue(buf, out);
            }
            if (n < sizeof(buf) - 1)
                buf[n++] = ch;
            start = millis();
        }
    }
    if (n) // Timeout: zeigen, was (unvollständig) ankam
        d7LogBytes("RX", (const uint8_t *)buf, n);
    return false; // Timeout → kein Wert / Sensor abgesteckt
}

bool AlmemoD7Sensor::parseValue(const char *s, float &out)
{
    if (*s == '=') // evtl. vorangestelltes Echo
        s++;

    /* In eine punkt-separierte Zahl umkopieren (deutsches Komma → Punkt),
     * Fremdzeichen ignorieren. */
    char   tmp[24];
    size_t j     = 0;
    bool   digit = false;
    for (; *s && j < sizeof(tmp) - 1; ++s) {
        char c = *s;
        if (c == ',')
            c = '.';
        if (c == '-' || c == '+' || c == '.' || (c >= '0' && c <= '9')) {
            tmp[j++] = c;
            if (c >= '0' && c <= '9')
                digit = true;
        }
    }
    tmp[j] = '\0';
    if (!digit)
        return false;

    out = (float)atof(tmp);
    return true;
}
