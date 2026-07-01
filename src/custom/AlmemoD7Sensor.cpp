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

bool AlmemoD7Sensor::probe_I2C(TwoWire &wire)
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
    uart      = &serial;
    fFormat   = 0;
    slotCount = 0;
    runSetup();
}

void AlmemoD7Sensor::runSetup()
{
    if (!uart)
        return;

    /* Eröffnungs-Session wie der Original-Logger, aus einzeln aufrufbaren
     * Kommandos zusammengesetzt. f4 ist Pflicht, damit '=' Momentanwerte liefert. */
    char buf[64];

    cmdHandshake(); // G-01

    if (cmdReadText(0, deviceName, sizeof(deviceName))) // t0 → Typ/Firmware
        LOG_INFO("AlmemoD7: %s", deviceName);

    cmdSetFormat(4);                   // f4 (Momentanwerte aktiv)
    cmdSetParam('k', 0);               // k0
    cmdSetParam('N', 2);               // N2
    cmdReadPage(15, buf, sizeof(buf)); // P15 — Messstellen-Legende (nur Log)

    /* Dimensionen der aktiven Kanäle aus der Geräte-Config bestimmen (P63/P65/P64)
     * — genau wie der Original-Logger, der daraus Einheit/Kommastellen kennt. */
    parseConfig();

    cmdReadPage(0, buf, sizeof(buf)); // P00 — Geräte-Ident (Session-Treue)

    LOG_INFO("AlmemoD7: UART setup done, %u channel(s) @%lu baud", slotCount, (unsigned long)UART_BAUD);
}

// =========================================================================
//  Config-Parsing: aktive Messkanäle → slots[]  (analog zum I2C-Treiber)
// =========================================================================

void AlmemoD7Sensor::parseConfig()
{
    slotCount = 0;
    if (!uart)
        return;

    char       p63[48], p65[256], p64[384];
    const bool ok63 = cmdReadPage(63, p63, sizeof(p63)); // aktive Messstellen
    const bool ok65 = cmdReadPage(65, p65, sizeof(p65)); // Messstelle→Bereich
    const bool ok64 = cmdReadPage(64, p64, sizeof(p64)); // Bereich→Einheit/Kommastellen
    if (!ok63 || !ok65 || !ok64) {
        LOG_ERROR("AlmemoD7: config read failed (P63=%d P65=%d P64=%d) → config aborted", ok63, ok65, ok64);
        return;
    }

    /* Aktive Messstellen aus P63 "i\<liste>" (z. B. "0,1" → Messstellen 0 und 1).
     * Genau diese Kanäle liefert '=' in derselben Reihenfolge. */
    size_t      ilen  = 0;
    const char *ilist = findValue(p63, "i", &ilen);
    if (!ilist) {
        LOG_ERROR("AlmemoD7: P63 without active-channel list → config aborted");
        return;
    }

    for (const char *s = ilist; s < ilist + ilen && slotCount < MAX_SLOTS;) {
        if (*s < '0' || *s > '9') {
            s++;
            continue;
        }
        uint8_t mst = 0;
        while (s < ilist + ilen && *s >= '0' && *s <= '9')
            mst = (uint8_t)(mst * 10 + (*s++ - '0'));

        /* Slot vollständig in ein lokales AlmemoSlot parsen; jeder fehlende
         * Baustein (Bereich in P65, Einheit+Kommastellen in P64) bricht die
         * gesamte Config ab — kein Default für Einheit/Exponent. */
        AlmemoSlot parsed;
        parsed.valueIndex = mst; // Messstellen-Nummer → Paket-valueIndex

        char code[12];
        if (!bereichForMessstelle(p65, mst, code, sizeof(code))) {
            LOG_ERROR("AlmemoD7: Messstelle %u not in P65 → config aborted", mst);
            slotCount = 0;
            return;
        }
        if (!bereichUnitExp(p64, code, parsed.unit, parsed.exponent)) {
            LOG_ERROR("AlmemoD7: Bereich '%s' (Mst %u) unit/exp missing in P64 → config aborted", code, mst);
            slotCount = 0;
            return;
        }

        char ustr[8]; // Einheit lesbar (0xF8→°) statt roher Hex-Bytes
        decodeToUtf8(parsed.unit, 2, ustr, sizeof(ustr));
        LOG_INFO("AlmemoD7: slot %u = Mst %u unit='%s' exp=%d", slotCount, mst, ustr, parsed.exponent);

        slots[slotCount++] = parsed; // erst jetzt, vollständig geparst, committen
    }
    LOG_INFO("AlmemoD7: %u active channel(s)", slotCount);
}

bool AlmemoD7Sensor::bereichForMessstelle(const char *p65, uint8_t mst, char *codeBuf, size_t codeCap)
{
    /* P65 ist eine Liste CRLF-getrennter Records "M\<0.N>;B\<code>". Pro Record
     * (nicht-destruktiv in eine lokale Kopie) die Tokens M und B auslesen. */
    for (const char *rec = p65; rec && *rec;) {
        const char *nl     = strstr(rec, "\r\n");
        size_t      reclen = nl ? (size_t)(nl - rec) : strlen(rec);
        char        line[48];
        size_t      cp = reclen < sizeof(line) - 1 ? reclen : sizeof(line) - 1;
        memcpy(line, rec, cp);
        line[cp] = '\0';

        size_t      mlen = 0, blen = 0;
        const char *mval = findValue(line, "M", &mlen);
        const char *bval = findValue(line, "B", &blen);
        if (mval && bval) {
            /* Messstellen-Nummer = Ziffern nach dem letzten '.' (Format "0.N"). */
            const char *p = mval;
            for (size_t k = 0; k < mlen; k++)
                if (mval[k] == '.')
                    p = mval + k + 1;
            uint8_t num = 0;
            while (p < mval + mlen && *p >= '0' && *p <= '9')
                num = (uint8_t)(num * 10 + (*p++ - '0'));
            if (num == mst) {
                size_t n = blen < codeCap - 1 ? blen : codeCap - 1;
                memcpy(codeBuf, bval, n);
                codeBuf[n] = '\0';
                return true;
            }
        }
        if (!nl)
            break;
        rec = nl + 2;
    }
    return false;
}

bool AlmemoD7Sensor::bereichUnitExp(const char *p64, const char *code, char unit[3], int8_t &exp)
{
    /* P64: CRLF-getrennte Records "B\<code>$<name>;1$\<Einheit>;2$\<..>;1K\<dez>;.." */
    for (const char *rec = p64; rec && *rec;) {
        const char *nl     = strstr(rec, "\r\n");
        size_t      reclen = nl ? (size_t)(nl - rec) : strlen(rec);
        char        line[128];
        size_t      cp = reclen < sizeof(line) - 1 ? reclen : sizeof(line) - 1;
        memcpy(line, rec, cp);
        line[cp] = '\0';

        size_t      blen = 0;
        const char *bval = findValue(line, "B", &blen);
        if (bval) {
            /* Bereich-Code = bval bis zum ersten '$' ("-01$DP04" → "-01"). */
            char   rc[12];
            size_t rn = 0;
            for (size_t k = 0; k < blen && bval[k] != '$' && rn < sizeof(rc) - 1; k++)
                rc[rn++] = bval[k];
            rc[rn] = '\0';
            if (strcmp(rc, code) == 0) {
                size_t      ulen = 0;
                const char *uval = findValue(line, "1$", &ulen); // Einheit
                const char *kval = findValue(line, "1K");        // Kommastellen
                if (!uval || !kval)
                    return false; // unvollständige Bereichsdefinition → kein gültiger Slot

                char   tmp[8];
                size_t tn = ulen < sizeof(tmp) - 1 ? ulen : sizeof(tmp) - 1;
                memcpy(tmp, uval, tn);
                tmp[tn] = '\0';
                normalizeUnit(tmp, unit);
                exp = (int8_t)(-atoi(kval)); // mehr Nachkommastellen → kleinerer Exponent
                return true;
            }
        }
        if (!nl)
            break;
        rec = nl + 2;
    }
    return false;
}

void AlmemoD7Sensor::normalizeUnit(const char *src, char out[3])
{
    size_t len = strlen(src);
    /* °C tritt je nach Seite als "##C", 0xF8+'C' oder "oC" auf → kanonisch auf
     * 0xF8 'C' abbilden (das erkennt der Empfänger als Temperatur). */
    if (len >= 2 && src[len - 1] == 'C') {
        uint8_t d = (uint8_t)src[0];
        if (d == '#' || d == 0xF8 || d == 'o' || d == 0xB0 || d == 0xDF) {
            out[0] = (char)0xF8;
            out[1] = 'C';
            out[2] = '\0';
            return;
        }
    }
    out[0] = len > 0 ? src[0] : ' ';
    out[1] = len > 1 ? src[1] : ' ';
    out[2] = '\0';
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
