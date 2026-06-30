#include "AlmemoD7Sensor.h"

#include "configuration.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace
{
/* UART-Verkehr lesbar nach LOG_DEBUG ausgeben: druckbares ASCII direkt, ETX als
 * <ETX>, die bekannten Hochbytes als ° / ü, alles andere als \xXX.
 * dir = "TX" (unsere Eingaben) / "RX" (Sensor). */
void d7LogBytes(const char *dir, const char *data, size_t len)
{
    char   out[200];
    size_t j         = 0;
    bool   truncated = false;
    for (size_t i = 0; i < len; i++) {
        if (j > sizeof(out) - 6) { // Platz fuer "\xXX" + NUL
            truncated = true;
            break;
        }
        uint8_t c = (uint8_t)data[i];
        if (c == AlmemoD7Sensor::ETX)
            j += snprintf(out + j, sizeof(out) - j, "<ETX>");
        else if (c == 0xF8) // Geräte-Codepage: Grad-Zeichen
            j += snprintf(out + j, sizeof(out) - j, "°");
        else if (c == 0xFC) // Geräte-Codepage: ü
            j += snprintf(out + j, sizeof(out) - j, "ü");
        else if (c >= 0x20 && c < 0x7F)
            out[j++] = (char)c;
        else
            j += snprintf(out + j, sizeof(out) - j, "\\x%02X", c);
    }
    out[j] = '\0';
    LOG_DEBUG("AlmemoD7 %s[%u]: %s%s", dir, (unsigned)len, out, truncated ? " …" : "");
}
} // namespace

// =========================================================================
//  I2C-Erkennung
// =========================================================================

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

// =========================================================================
//  Setup / Lebenszyklus
// =========================================================================

void AlmemoD7Sensor::beginUart(Stream &serial)
{
    uart    = &serial;
    fFormat = 0;
    runSetup();
}

void AlmemoD7Sensor::runSetup()
{
    if (!uart)
        return;

    /* Die komplette Eröffnungs-Session exakt wie der Original-Logger, jetzt aus
     * einzeln aufrufbaren Kommandos zusammengesetzt. Die Konfig-Seiten lesen wir
     * nur ein (Inhalt landet im Debug-Log); für den Messbetrieb wird nur '='
     * gebraucht. */
    char buf[256];

    cmdHandshake(); // G-01

    char type[40] = {0};
    if (cmdReadText(0, type, sizeof(type))) // t0 → Typ/Firmware
        LOG_INFO("AlmemoD7: %s", type);

    cmdSetFormat(4);                  // f4
    cmdSetParam('k', 0);              // k0
    cmdSetParam('N', 2);              // N2
    cmdReadPage(15, buf, sizeof(buf)); // P15 — Messstellen-Konfig
    cmdReadPage(64, buf, sizeof(buf)); // P64 — Bereichs-/Funktionskanal-Defs
    cmdReadPage(65, buf, sizeof(buf)); // P65 — Messstellen→Bereich-Map
    cmdSetFormat(1);                  // f1
    cmdReadPage(65, buf, sizeof(buf)); // P65 (anderes Format wg. f1)
    cmdSetFormat(4);                  // f4 — Format zurück
    cmdReadPage(0, buf, sizeof(buf));  // P00 — Geräte-Ident

    LOG_INFO("AlmemoD7: UART setup done @%lu baud", (unsigned long)UART_BAUD);
}

// =========================================================================
//  Low-Level-Primitive
// =========================================================================

void AlmemoD7Sensor::sendCommand(const char *cmd)
{
    if (!uart || !cmd)
        return;
    size_t len = strlen(cmd);
    uart->write((const uint8_t *)cmd, len); // kein Terminator anhängen!
    d7LogBytes("TX", cmd, len);
}

size_t AlmemoD7Sensor::readFrame(char *buf, size_t cap, uint32_t timeoutMs, bool *gotEtx)
{
    size_t n   = 0;
    bool   etx = false;
    if (uart) {
        uint32_t start = millis();
        while (millis() - start < timeoutMs) {
            while (uart->available()) {
                uint8_t c = uart->read();
                start     = millis(); // solange Bytes fließen, Timeout nachladen
                if (c == ETX) {       // Frame komplett — ETX nicht ablegen
                    etx = true;
                    break;
                }
                if (buf && cap && n < cap - 1) // überzählige Bytes weiter lesen, nur nicht speichern
                    buf[n++] = (char)c;
            }
            if (etx)
                break;
        }
    }
    if (buf && cap)
        buf[n] = '\0';
    if (gotEtx)
        *gotEtx = etx;
    if (n || etx)
        d7LogBytes("RX", buf ? buf : "", n);
    return n;
}

size_t AlmemoD7Sensor::exec(const char *cmd, char *buf, size_t cap, uint32_t timeoutMs, bool *gotEtx)
{
    sendCommand(cmd);
    return readFrame(buf, cap, timeoutMs, gotEtx);
}

// =========================================================================
//  High-Level-Kommandos
// =========================================================================

bool AlmemoD7Sensor::cmdHandshake(const char *addr, char *status)
{
    char buf[16];
    bool etx = false;
    exec(addr, buf, sizeof(buf), 150, &etx);
    if (status)
        *status = buf[0];
    return etx && buf[0] == '1'; // Handshake: Statusziffer '1' = OK
}

bool AlmemoD7Sensor::cmdReadText(uint8_t field, char *out, size_t outLen)
{
    if (!out || !outLen)
        return false;
    out[0] = '\0';

    char cmd[6];
    snprintf(cmd, sizeof(cmd), "t%u", field);
    char buf[64];
    bool etx = false;
    exec(cmd, buf, sizeof(buf), 150, &etx);
    if (!etx)
        return false;

    /* Config-Frame: erste Zeile = Echo, danach die Nutzlast-Zeile (Typ/Firmware),
     * rechts mit Leerzeichen aufgefüllt → trailing spaces trimmen. */
    char *nl  = strstr(buf, "\r\n");
    char *p   = nl ? nl + 2 : buf;
    char *end = strstr(p, "\r\n");
    size_t len = end ? (size_t)(end - p) : strlen(p);
    while (len && p[len - 1] == ' ')
        len--;

    return decodeToUtf8(p, len, out, outLen) > 0 || len == 0;
}

bool AlmemoD7Sensor::cmdSetFormat(uint8_t mode)
{
    char cmd[6];
    snprintf(cmd, sizeof(cmd), "f%u", mode);
    bool etx = false;
    exec(cmd, nullptr, 0, 150, &etx);
    if (etx)
        fFormat = mode; // f-Zustand mitführen (beeinflusst u. a. P65)
    return etx;
}

bool AlmemoD7Sensor::cmdSetParam(char letter, uint8_t value)
{
    char cmd[6];
    snprintf(cmd, sizeof(cmd), "%c%u", letter, value);
    bool etx = false;
    exec(cmd, nullptr, 0, 150, &etx);
    return etx;
}

bool AlmemoD7Sensor::cmdReadPage(uint8_t page, char *buf, size_t cap, uint32_t timeoutMs)
{
    if (!buf || !cap)
        return false;
    buf[0] = '\0';

    char cmd[6];
    snprintf(cmd, sizeof(cmd), "P%02u", page);
    bool etx = false;
    exec(cmd, buf, cap, timeoutMs, &etx);
    if (!etx)
        return false;

    /* Echo-Zeile vorne und abschließendes CR/LF entfernen → reine Nutzlast. */
    char *nl      = strstr(buf, "\r\n");
    char *payload = nl ? nl + 2 : buf;
    size_t len    = strlen(payload);
    while (len && (payload[len - 1] == '\r' || payload[len - 1] == '\n'))
        payload[--len] = '\0';
    if (payload != buf)
        memmove(buf, payload, len + 1);
    return true;
}

size_t AlmemoD7Sensor::cmdMeasure(float *out, size_t maxOut)
{
    if (!uart || !out || !maxOut)
        return 0;

    while (uart->available()) // alte/Restbytes verwerfen
        uart->read();

    char buf[48];
    bool etx = false;
    exec("=", buf, sizeof(buf), 120, &etx); // Measurement: Wandlung → großzügiger Timeout
    if (!etx)
        return 0; // Timeout → Sensor abgesteckt
    return parseValues(buf, out, maxOut);
}

bool AlmemoD7Sensor::readValue(float &out)
{
    float vals[4];
    size_t n = cmdMeasure(vals, 4);
    if (!n)
        return false;
    out = vals[0]; // ersten aktiven Kanal liefern
    return true;
}

// =========================================================================
//  Diagnose
// =========================================================================

void AlmemoD7Sensor::dumpPages(uint8_t from, uint8_t to, uint32_t timeoutMs)
{
    if (!uart)
        return;

    char raw[256]; // Roh-Nutzlast (cmdReadPage liefert Echo/CRLF schon entfernt)
    char txt[300]; // dekodiert nach UTF-8 fürs Log
    for (uint8_t mode = 1; mode <= 4; mode++) {
        cmdSetFormat(mode);
        LOG_INFO("AlmemoD7: ===== Seiten P%02u..P%02u @ f%u =====", from, to, mode);
        for (uint8_t p = from;; p++) {
            bool ok = cmdReadPage(p, raw, sizeof(raw), timeoutMs);
            decodeToUtf8(raw, strlen(raw), txt, sizeof(txt));
            LOG_INFO("AlmemoD7 P%02u@f%u%s: %s", p, mode, ok ? "" : " [Timeout]", txt);
            if (p == to) // so terminiert auch to == 255 ohne uint8-Überlauf
                break;
        }
    }
    cmdSetFormat(4); // Ausgangszustand wiederherstellen
}

void AlmemoD7Sensor::dumpTextFields(uint8_t from, uint8_t to)
{
    if (!uart)
        return;
    char txt[64];
    for (uint8_t f = from;; f++) {
        if (cmdReadText(f, txt, sizeof(txt)))
            LOG_INFO("AlmemoD7 t%u: \"%s\"", f, txt);
        else
            LOG_INFO("AlmemoD7 t%u: [Timeout]", f);
        if (f == to)
            break;
    }
}

// =========================================================================
//  Statische Parse-Helfer
// =========================================================================

size_t AlmemoD7Sensor::parseValues(const char *body, float *out, size_t maxOut)
{
    size_t      cnt = 0;
    const char *s   = body;
    while (*s && cnt < maxOut) {
        float v;
        if (parseValue(s, v)) // liest die erste Zahl bis zum ';'
            out[cnt++] = v;
        while (*s && *s != SEP) // bis zum nächsten Trenner vorrücken
            s++;
        if (*s == SEP)
            s++;
    }
    return cnt;
}

bool AlmemoD7Sensor::splitToken(char *token, char **key, char **value)
{
    if (!token)
        return false;
    char *bs = strchr(token, KV); // erstes '\'
    if (bs) {
        *bs    = '\0';
        *value = bs + 1;
    } else {
        *value = nullptr; // Katalogzeile: nur Key, kein '\'
    }
    *key = token;
    return token[0] != '\0';
}

const char *AlmemoD7Sensor::findValue(const char *line, const char *key, size_t *len)
{
    if (!line || !key)
        return nullptr;
    size_t klen = strlen(key);
    const char *p = line;
    while (*p) {
        const char *tokEnd = strchr(p, SEP);
        size_t      tokLen = tokEnd ? (size_t)(tokEnd - p) : strlen(p);
        /* Token muss mit "<key>\" beginnen. */
        if (tokLen > klen && memcmp(p, key, klen) == 0 && p[klen] == KV) {
            const char *val = p + klen + 1;
            if (len)
                *len = tokLen - klen - 1;
            return val;
        }
        if (!tokEnd)
            break;
        p = tokEnd + 1;
    }
    return nullptr;
}

size_t AlmemoD7Sensor::decodeToUtf8(const char *in, size_t len, char *out, size_t outCap)
{
    if (!out || !outCap)
        return 0;
    size_t j = 0;
    for (size_t i = 0; i < len && j < outCap - 1; i++) {
        uint8_t c = (uint8_t)in[i];
        if (c == 0xF8 && j < outCap - 2) { // ° → U+00B0
            out[j++] = (char)0xC2;
            out[j++] = (char)0xB0;
        } else if (c == 0xFC && j < outCap - 2) { // ü → U+00FC
            out[j++] = (char)0xC3;
            out[j++] = (char)0xBC;
        } else if (c >= 0x20 && c < 0x7F) {
            out[j++] = (char)c;
        } else {
            out[j++] = '?';
        }
    }
    out[j] = '\0';
    return j;
}

bool AlmemoD7Sensor::parseValue(const char *s, float &out)
{
    if (*s == '=') // evtl. vorangestelltes Echo
        s++;

    /* Die Messantwort kann mehrere Kanäle führen ("26,44;26,44") — nur den
     * ersten Wert bis zum ';' auswerten. In eine punkt-separierte Zahl
     * umkopieren (deutsches Komma → Punkt), Fremdzeichen ignorieren. */
    char   tmp[24];
    size_t j     = 0;
    bool   digit = false;
    for (; *s && *s != ';' && j < sizeof(tmp) - 1; ++s) {
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
