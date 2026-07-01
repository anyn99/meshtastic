#pragma once

#include "AlmemoCommon.h"
#include <Arduino.h>
#include <Wire.h>
#include <stddef.h>
#include <stdint.h>

/**
 * AlmemoD7Sensor — Treiber für den ALMEMO-D7-Sensor (z. B. ZPD700-FS).
 *
 * Erkennung über I2C (Typ-Byte 0x7B an 0x50/Reg 0x08), danach läuft die ganze
 * Kommunikation über UART (Serial1, D6=TX/D7=RX) im ALMEMO-ASCII-Protokoll.
 *
 * Protokoll (rekonstruiert aus einem vollständigen Bus-Mitschnitt):
 *
 *   Physik:   921600 Baud, 8N1, Halbduplex, strikt Master→Slave (Request→Response).
 *
 *   Request:  nur die ASCII-Kommandobytes, OHNE jeden Abschluss (kein CR/LF/ETX).
 *             Argumentlängen sind pro Kommandobuchstabe fest (siehe Cmd-Tabelle).
 *
 *   Response: drei Frame-Arten, alle enden mit ETX (0x03) — die einzige
 *             verlässliche Frame-Grenze:
 *               Config      (t/f/k/N/P..):  <ECHO>␍␊[<Zeile>(␍␊<Zeile>)*␍␊]␃
 *               Measurement ('='):          <Wert1>;<Wert2>;…␃   (kein Echo/CRLF)
 *               Handshake   ('G-01'):       <Statusziffer>␍␊␃
 *
 *   Typ-A-Nutzlast: Zeilen an ␍␊; je Zeile Tokens an ';'; je Token am ERSTEN '\'
 *                   (0x5C) in key/value. '$' niemals global splitten (kommt in
 *                   Keys 1$/2$ und in B-Werten <Code>$<Kurzname> vor).
 *
 *   Codepage:  gerätespezifisch (Ahlborn), nicht Latin-1. Nur Hochbytes
 *              0xF8 = '°' und 0xFC = 'ü' beobachtet → eigene Lookup-Tabelle.
 *
 *   Beispiel-Session: G-01 → t0 → f4 → k0 → N2 → P15 → P64 → P65 → f1 → P65 →
 *                     f4 → P00, danach zyklisch (1 s) '=' pollen.
 */
class AlmemoD7Sensor
{
  public:
    // ---- I2C-Erkennung -----------------------------------------------------
    static constexpr uint8_t  I2C_ADDR  = 0x50; /* I2C-Erkennungs-Adresse */
    static constexpr uint8_t  REG_TYPE  = 0x08; /* Register mit dem Typ-Byte */
    static constexpr uint8_t  TYPE_D7   = 0x7B; /* Kennung eines ALMEMO D7 */
    static constexpr uint32_t UART_BAUD = 921600;

    // ---- Protokoll-Steuerzeichen ------------------------------------------
    static constexpr uint8_t CR  = 0x0D;
    static constexpr uint8_t LF  = 0x0A;
    static constexpr uint8_t ETX = 0x03;
    static constexpr char    KV  = '\\'; /* Key/Value-Trenner (0x5C) */
    static constexpr char    SEP = ';';  /* Token-/Wert-Trenner */

    /* Der D7 (ZPD700-FS) führt bis zu 10 Messstellen (0..9) — siehe P19 "M10" /
     * P70 "M0010" und die P65-Liste M\0.0..0.9. Dank 4-Bit-valueIndex im Paket
     * (AlmemoValue::slot, seit v3) passen alle 10 unter einen channel. Die aktiven
     * Kanäle liegen als kompakte AlmemoSlot-Liste vor (valueIndex = Messstelle). */
    static constexpr uint8_t MAX_SLOTS = 10;

    /**
     * I2C-Erkennungs-Handshake.
     * @return true wenn an 0x50 ein ALMEMO D7 antwortet (Typ-Byte == 0x7B).
     */
    static bool probe_I2C(TwoWire &wire);

    /**
     * UART übernehmen und einmalig die Setup-Sequenz fahren. Der Aufrufer muss
     * den Port vorher öffnen (serial.begin(UART_BAUD)).
     */
    void beginUart(Stream &serial);

    /**
     * Einen Messwert abfragen ('='): ersten Kanal der Antwort liefern.
     * @return true + Wert in out; false bei Timeout (Sensor weg) / Parse-Fehler.
     */
    bool readValue(float &out);

    // =======================================================================
    //  Low-Level-Primitive — einzeln aufrufbar
    // =======================================================================

    /** Kommando-ASCII ohne Terminator senden (kein CR/LF/ETX angehängt). */
    void sendCommand(const char *cmd);

    /**
     * Einen Frame bis einschließlich ETX lesen. ETX wird NICHT in buf abgelegt;
     * buf wird nullterminiert. CR/LF bleiben erhalten.
     * @param gotEtx (optional) true, wenn ETX gesehen wurde (sonst Timeout).
     * @param truncated (optional) true, wenn der Frame länger als der Puffer war
     *        und hinten abgeschnitten wurde (Bytes verworfen).
     * @return Anzahl gespeicherter Nutzbytes (ohne ETX), geklammert auf cap-1.
     */
    size_t readFrame(char *buf, size_t cap, uint32_t timeoutMs, bool *gotEtx = nullptr, bool *truncated = nullptr);

    /**
     * Kommando senden und Antwort-Frame lesen (sendCommand + readFrame).
     * @return wie readFrame; *gotEtx zeigt einen kompletten Frame an.
     */
    size_t exec(const char *cmd, char *buf, size_t cap, uint32_t timeoutMs, bool *gotEtx = nullptr,
                bool *truncated = nullptr);

    // =======================================================================
    //  High-Level-Kommandos — einzeln aufrufbar
    // =======================================================================

    /** G-01: Geräteauswahl/Handshake. @return true bei Status '1'. status: Statusziffer. */
    bool cmdHandshake(const char *addr = "G-01", char *status = nullptr);

    /** t<field>: Textfeld lesen (Typ/Firmware). out wird getrimmt nullterminiert. */
    bool cmdReadText(uint8_t field, char *out, size_t outLen);

    /** f<mode>: Ausgabeformat wählen (führt den f-Zustand mit). */
    bool cmdSetFormat(uint8_t mode);

    /** k<v> / N<v>: einstelligen Parameter setzen (letter = 'k' | 'N' | …). */
    bool cmdSetParam(char letter, uint8_t value);

    /**
     * P<nn>: Konfigurationsseite lesen. Roh-Nutzlast (Echo/CRLF/ETX entfernt) in
     * buf nullterminiert ablegen — zum Weiterverarbeiten mit den Token-Helfern.
     * @param truncated (optional) true, wenn die Seite größer als cap war und
     *        abgeschnitten wurde → Puffer vergrößern.
     */
    bool cmdReadPage(uint8_t page, char *buf, size_t cap, uint32_t timeoutMs = 200, bool *truncated = nullptr);

    /**
     * '=': aktuelle Messwerte abrufen. Alle Kanäle nach out (max maxOut),
     * Komma→Punkt. @return Anzahl geparster Werte (0 = Timeout/Fehler).
     */
    size_t cmdMeasure(float *out, size_t maxOut);

    /** Aktueller f-Zustand (1/4); beeinflusst u. a. das Format von P65. */
    uint8_t format() const { return fFormat; }

    // =======================================================================
    //  Messkanäle / Slots — analog zum ALMEMO-I2C-Treiber
    // =======================================================================

    /** Anzahl der beim Setup erkannten aktiven Messkanäle (0..MAX_SLOTS). */
    uint8_t numSlots() const { return slotCount; }

    /** Metadaten (valueIndex/Einheit/Exponent) des i-ten aktiven Kanals; i < numSlots(). */
    const AlmemoSlot &slot(uint8_t i) const { return slots[i]; }

    // =======================================================================
    //  Diagnose — zum Erkunden unbekannter Seiten/Felder (rein lesend)
    // =======================================================================

    /**
     * Alle Seiten P<from>..P<to> unter JEDEM Ausgabeformat f1..f4 abfragen und
     * dekodiert per LOG_INFO ausgeben — um zu sehen, welche Seiten existieren und
     * wie sich ihr Inhalt mit dem f-Format ändert. Stellt am Ende f4 wieder her.
     *
     * Achtung: BLOCKIEREND. Bei (to-from+1)·4 Abfragen × timeoutMs kann das viele
     * Sekunden dauern → nur als einmalige Diagnose aufrufen, nicht im runOnce-
     * Takt, und ggf. den Bereich eingrenzen (Watchdog!).
     */
    void dumpPages(uint8_t from = 0, uint8_t to = 99, uint32_t timeoutMs = 80);

    /** Textfelder t<from>..t<to> abfragen und per LOG_INFO ausgeben (rein lesend). */
    void dumpTextFields(uint8_t from = 0, uint8_t to = 9);

    // =======================================================================
    //  Statische Parse-Helfer
    // =======================================================================

    /**
     * Typ-B-Body ("26,44;26,44") in Floats zerlegen. Komma→Punkt, Split an ';'.
     * @return Anzahl geparster Werte (≤ maxOut).
     */
    static size_t parseValues(const char *body, float *out, size_t maxOut);

    /**
     * Ein Token "<key>\<value>" am ERSTEN '\' trennen. Arbeitet in-place:
     * token wird am '\' auf '\0' gesetzt. value zeigt hinter das '\', oder ist
     * nullptr (Katalogzeile ohne '\'). @return true bei vorhandenem Key.
     */
    static bool splitToken(char *token, char **key, char **value);

    /**
     * In einer einzelnen Nutzlastzeile den Wert zu key suchen (z. B. "1K", "B").
     * Nicht-destruktiv. @return Zeiger auf den Wert-Anfang oder nullptr.
     * len liefert (optional) die Wertlänge bis zum nächsten ';'.
     */
    static const char *findValue(const char *line, const char *key, size_t *len = nullptr);

    /**
     * Geräte-Codepage → UTF-8. Druckbares ASCII 1:1, 0xF8→"°", 0xFC→"ü",
     * sonstige Steuer-/Hochbytes → '?'. out wird nullterminiert.
     * @return Anzahl geschriebener Bytes (ohne NUL).
     */
    static size_t decodeToUtf8(const char *in, size_t len, char *out, size_t outCap);

  private:
    Stream    *uart    = nullptr;
    uint8_t    fFormat = 0; /* zuletzt per f<mode> gesetztes Ausgabeformat */
    AlmemoSlot slots[MAX_SLOTS]{};
    uint8_t    slotCount = 0;
    char       deviceName[24]{}; /* Typ/Firmware aus t0 (für Logs) */

    void runSetup();

    /**
     * Aktive Messkanäle aus der Geräte-Config einlesen (P63/P65/P64) und slots[]
     * füllen. Alles-oder-nichts: kann eine Messstelle nicht vollständig aufgelöst
     * werden (Bereich/Einheit/Kommastellen), wird mit LOG_ERROR abgebrochen und
     * slotCount bleibt 0 — keine Default-Einheit/-Exponent.
     */
    void parseConfig();

    /* P65: Bereich-Code der Messstelle mst nach codeBuf (z. B. "-01"). */
    static bool bereichForMessstelle(const char *p65, uint8_t mst, char *codeBuf, size_t codeCap);
    /* P64: Einheit + Exponent (aus 1$/1K) des Bereichs code. */
    static bool bereichUnitExp(const char *p64, const char *code, char unit[3], int8_t &exp);
    /* Config-Einheit ("##C"/0xF8C/"oC"/"V") in eine paketfertige 2-Byte-Einheit normieren. */
    static void normalizeUnit(const char *src, char out[3]);

    static bool parseValue(const char *s, float &out);
};
