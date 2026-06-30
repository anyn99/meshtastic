#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

/**
 * AlmemoD7Sensor — ALMEMO-D7-Sensor.
 *
 * Erkennung über I2C (Typ-Byte 0x7B an 0x50/Reg 0x08), danach läuft die ganze
 * Kommunikation über UART (Serial1, D6=TX/D7=RX) im ALMEMO-ASCII-Protokoll:
 *
 *   921600 Baud, 8N1.
 *   - Einmalig nach dem Erkennen eine Setup-Sequenz fahren (Geräte-Adressierung
 *     + Kommandos t0/f4/k0/N2/P15/P64/P65/f1/P65/f4/P00). Antworten enden mit
 *     ETX (0x03).
 *   - Danach pro Messwert ein '=' senden; der Sensor antwortet mit dem Wert als
 *     ASCII (deutsches Dezimalkomma) + ETX, z. B. "26,37\x03" → 26.37 °C.
 */
class AlmemoD7Sensor
{
  public:
    static constexpr uint8_t  I2C_ADDR  = 0x50; /* I2C-Erkennungs-Adresse */
    static constexpr uint8_t  REG_TYPE  = 0x08; /* Register mit dem Typ-Byte */
    static constexpr uint8_t  TYPE_D7   = 0x7B; /* Kennung eines ALMEMO D7 */
    static constexpr uint32_t UART_BAUD = 921600;

    /**
     * I2C-Erkennungs-Handshake.
     * @return true wenn an 0x50 ein ALMEMO D7 antwortet (Typ-Byte == 0x7B).
     */
    static bool probe(TwoWire &wire);

    /**
     * UART übernehmen und einmalig die Setup-Sequenz fahren. Der Aufrufer muss
     * den Port vorher öffnen (serial.begin(UART_BAUD)).
     */
    void beginUart(Stream &serial);

    /**
     * Einen Messwert abfragen: '=' senden, Antwort bis ETX lesen und parsen.
     * @return true + skalierter Wert in out; false bei Timeout (Sensor weg).
     */
    bool readValue(float &out);

  private:
    Stream *uart = nullptr;

    void        runSetup();
    bool        drainUntilEtx(uint32_t timeoutMs);
    static bool parseValue(const char *s, float &out);
};
