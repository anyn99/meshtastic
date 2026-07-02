#pragma once

#include "concurrency/OSThread.h"
#include "configuration.h"
#include "power/PowerHAL.h"
#include <Arduino.h>

/**
 * AlmemoLedThread
 *
 * Treibt die RGB-LED des Sender-Boards (common anode, active LOW) als
 * Status-Anzeige. Die Blink-Zeiten werden hier erzeugt, entkoppelt vom
 * AlmemoSenderThread (der bis zu mehreren Sekunden schläft und daher kein
 * 2/998-ms-Blinken treiben könnte).
 *
 *   - BLAU  : kurzer Puls beim Senden
 *   - GRÜN  : dauerhaft, solange USB angesteckt ist — auch zwischen den Blinks
 *   - GELB  : deutlicher Blitz mit langer Pause, wenn kein Sensor gefunden wurde
 *   - ROT   : deutlicher Blitz mit langer Pause, bei sonstigem Problem (z. B. Lesefehler)
 *   - AUS   : nur ohne USB / schlafend
 *
 * Der Farb-Blitz ist lang genug, um neben dem Dauer-Grün (USB) gut sichtbar zu
 * sein; die Pause dazwischen ist bewusst lang, damit im Batteriebetrieb (LED in
 * der Pause aus) wenig Strom verbraucht wird. Bei USB bleibt die Pause grün.
 *
 * Status wird vom AlmemoSenderThread über setState()/pulseSend() gesetzt.
 */

#ifndef ALMEMO_LED_BLINK_ON_MS
#define ALMEMO_LED_BLINK_ON_MS 300
#endif

#ifndef ALMEMO_LED_BLINK_OFF_MS
#define ALMEMO_LED_BLINK_OFF_MS 3000
#endif

// Wie lange Blau nach einem Sendevorgang sichtbar gehalten wird.
// MUSS größer als ALMEMO_LED_IDLE_POLL_MS sein, sonst verschläft der Thread den Puls.
#ifndef ALMEMO_LED_SEND_HOLD_MS
#define ALMEMO_LED_SEND_HOLD_MS 150
#endif

// Poll-Intervall im Ruhezustand (LED aus). Klein genug, um jeden Sende-Puls
// zuverlässig zu erwischen (< ALMEMO_LED_SEND_HOLD_MS).
#ifndef ALMEMO_LED_IDLE_POLL_MS
#define ALMEMO_LED_IDLE_POLL_MS 50
#endif

enum class AlmemoLedState : uint8_t {
    Idle,     // nichts anzuzeigen -> LED aus
    NoSensor, // kein Sensor erkannt -> gelb blinken
    Error,    // Lese-/sonstiger Fehler -> rot blinken
};

class AlmemoLedThread : public concurrency::OSThread
{
  public:
    AlmemoLedThread() : OSThread("AlmemoLed")
    {
        pinMode(LED_RED, OUTPUT);
        pinMode(LED_GREEN, OUTPUT);
        pinMode(LED_BLUE, OUTPUT);
        allOff();
    }

    /** Persistenten Anzeige-Zustand setzen (idle / kein Sensor / Fehler). */
    void setState(AlmemoLedState s) { state = s; }

    /**
     * Blauen Sende-Puls auslösen bzw. verlängern (jeder Aufruf hält Blau ALMEMO_LED_SEND_HOLD_MS).
     * Schaltet Blau sofort ein, damit der Flash auch dann sichtbar ist, wenn der LED-Thread
     * gerade verdrängt wird (z. B. direkt nach dem Boot). Das Ausschalten erledigt runOnce().
     */
    void pulseSend()
    {
        lastSendMs = millis();
        setRGB(false, false, true); // blau
    }

    /** LED sofort ausschalten (z. B. vor Deep Sleep). */
    void off()
    {
        state = AlmemoLedState::Idle;
        lastSendMs = 0;
        allOff();
    }

  protected:
    int32_t runOnce() override
    {
        const uint32_t now = millis();

        // Blau hat Vorrang: kurzer solider Puls rund um jeden Sendevorgang.
        const uint32_t sinceSend = now - lastSendMs;
        if (lastSendMs != 0 && sinceSend < ALMEMO_LED_SEND_HOLD_MS) {
            setRGB(false, false, true); // blau
            return ALMEMO_LED_SEND_HOLD_MS - sinceSend;
        }

        switch (state) {
        case AlmemoLedState::NoSensor:
            return blink(true, true, false); // gelb = rot + grün
        case AlmemoLedState::Error:
            return blink(true, false, false); // rot
        default:
            // Idle: dauerhaft grün, solange USB (VBUS) anliegt; sonst aus.
            setRGB(false, powerHAL_isVBUSConnected(), false);
            return ALMEMO_LED_IDLE_POLL_MS;
        }
    }

  private:
    // 2 ms Farbe AN / 998 ms "Aus". In der Aus-Phase grün, solange USB (VBUS) anliegt,
    // damit die LED durchgehend grün leuchtet und nur kurz rot/gelb blitzt.
    int32_t blink(bool r, bool g, bool b)
    {
        blinkOn = !blinkOn;
        if (blinkOn) {
            setRGB(r, g, b);
            return ALMEMO_LED_BLINK_ON_MS;
        }
        setRGB(false, powerHAL_isVBUSConnected(), false);
        return ALMEMO_LED_BLINK_OFF_MS;
    }

    static void writeLed(uint8_t pin, bool on) { digitalWrite(pin, on ? LED_STATE_ON : !LED_STATE_ON); }

    static void setRGB(bool r, bool g, bool b)
    {
        writeLed(LED_RED, r);
        writeLed(LED_GREEN, g);
        writeLed(LED_BLUE, b);
    }

    static void allOff() { setRGB(false, false, false); }

    AlmemoLedState state = AlmemoLedState::Idle;
    bool blinkOn = false;
    uint32_t lastSendMs = 0;
};

extern AlmemoLedThread *almemoLedThread;
