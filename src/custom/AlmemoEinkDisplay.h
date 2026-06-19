#pragma once

#include "configuration.h"

#if defined(ALMEMO_EINK)

#include "SPILock.h"

#include <GxEPD2_BW.h>
#include <SPI.h>

/*
 * AlmemoEinkDisplay
 *
 * WeAct Studio 1.54" 200x200 E-Paper (Controller SSD1681 -> GxEPD2-Klasse
 * GxEPD2_154_D67). KEIN eigener Thread: das Display wird synchron aktualisiert,
 * indem der AlmemoSenderThread almemoEinkPublish() aufruft (nach jedem Senden
 * bzw. bei Sensorwechsel). Es wird also nur gezeichnet, wenn sich etwas ändert.
 *
 * Hardware: SPI-Bus geteilt mit dem LoRa-Radio (SCK=D8, MOSI=D10; MISO ungenutzt).
 * RES braucht einen echten GPIO mit Reset-Puls (Pins via platformio.ini-Flags).
 *
 * SPI-Zugriff läuft unter dem Firmware-spiLock, ABER der ~2.6s Full-Refresh wird
 * asynchron gefahren: Lock nur um die kurzen SPI-Schreibvorgänge, das BUSY-Warten
 * läuft lock-frei (epd2.isBusy()), damit der Radio-Task den Bus nicht verliert.
 *
 * Layout:
 *   - obere zwei Drittel: 4 nummerierte Rechtecke (2x2, ~20:8).
 *       hinten (obere Zeile) = 1 / 2, vorne (untere Zeile) = 3 / 4.
 *   - unteres Drittel: aktueller Sendeintervall + letzter Timestamp.
 */

#ifndef ALMEMO_EINK_CS
#define ALMEMO_EINK_CS 19
#endif
#ifndef ALMEMO_EINK_DC
#define ALMEMO_EINK_DC 20
#endif
#ifndef ALMEMO_EINK_BUSY
#define ALMEMO_EINK_BUSY 21
#endif
#ifndef ALMEMO_EINK_RST
#define ALMEMO_EINK_RST -1
#endif

// Pro Rechteck (Sensorposition hinten 1/2, vorne 3/4) anzuzeigende Info.
struct AlmemoEinkSensorInfo {
    bool present;        // false -> Rechteck zeigt nur die Position
    uint8_t valueCount;  // Anzahl Messwerte (max 4)
    char name[9];        // Sensorname, 8 ASCII + NUL
};

class AlmemoEinkDisplay
{
    // SPI (global, geteilt mit dem LoRa-Radio) wird in diesem GxEPD2-Fork im Konstruktor gebunden.
    GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> display{
        GxEPD2_154_D67(ALMEMO_EINK_CS, ALMEMO_EINK_DC, ALMEMO_EINK_RST, ALMEMO_EINK_BUSY, SPI)};

    bool inited = false;

    uint32_t intervalSecs = 0;  // aktueller Sendeintervall in Sekunden
    uint32_t lastTimestamp = 0; // Unix-Sekunden des letzten Sendens (0 = unbekannt)
    AlmemoEinkSensorInfo sensors[4] = {}; // 4 Rechtecke: hinten 1/2, vorne 3/4

  public:
    // Daten übernehmen und das Display sofort neu zeichnen.
    // Wird synchron aus dem AlmemoSenderThread aufgerufen.
    void publish(uint32_t intervalSecs, uint32_t unixTimestamp, const AlmemoEinkSensorInfo sensorsIn[4])
    {
        this->intervalSecs = intervalSecs;
        this->lastTimestamp = unixTimestamp;
        for (int i = 0; i < 4; i++)
            this->sensors[i] = sensorsIn[i];

        if (!inited) {
            // init() macht Reset + Controller-Setup (kurze SPI-Phase) -> Lock nur hier.
            concurrency::LockGuard g(spiLock);
            display.init(0, true, 10, false);
            display.setRotation(3); // 180° (Modul-Einbaulage, Bild stand sonst auf dem Kopf)
            inited = true;
        }
        render();
    }

  private:
    // Immer Full-Refresh, aber ASYNCHRON: Der ~2.6s BUSY-Wait darf den geteilten
    // SPI-Bus nicht blockieren, sonst läuft die Tx-Nachbereitung des Radios
    // (setStandby) in den spiLock-Timeout (RadioLib err=-705 -> assert).
    // Mit -DUSE_EINK_DYNAMICDISPLAY überspringt nextPage() das interne BUSY-Warten;
    // wir warten selbst über epd2.isBusy() (nur digitalRead, kein SPI) OHNE Lock.
    void render()
    {
        // 1) Bild in den RAM-Framebuffer + Full-Refresh anstoßen (kurzer Lock, nur SPI-Schreiben).
        {
            concurrency::LockGuard g(spiLock);
            display.setFullWindow();
            display.firstPage();
            do {
                display.fillScreen(GxEPD_WHITE);
                display.setTextColor(GxEPD_BLACK);
                drawContents();
            } while (display.nextPage()); // async: startet Refresh, kehrt sofort zurück
        }

        // 2) Auf den Panel-Refresh warten OHNE Lock -> Radio kann den Bus nutzen.
        while (display.epd2.isBusy())
            delay(10);

        // 3) Abschluss (writeImageAgain + powerOff) -> wieder kurzer Lock.
        {
            concurrency::LockGuard g(spiLock);
            display.endAsyncFull();
        }
    }

    void drawContents()
    {
        const int W = display.width();  // 200
        const int H = display.height(); // 200
        const int topH = (H * 2) / 3;   // 133 -> obere zwei Drittel

        // --- obere zwei Drittel: 4 Rechtecke (2x2), Positionsnummer AUSSERHALB
        //     oben links über jedem Rechteck; im Rechteck "Name (Anzahl)" in Size 2.
        const int margin = 4;
        const int colGap = 6;
        const int rectW = (W - 2 * margin - colGap) / 2; // 93
        const int numH = 16;                             // Platz über dem Rechteck für die Nummer (Size 2)
        const int rectH = 40;
        const int rowGap = 6;

        const int blockH = 2 * (numH + rectH) + rowGap;
        const int yTop = (topH - blockH) / 2; // vertikal im oberen Bereich zentriert

        const int xs[2] = {margin, margin + rectW + colGap};
        const int numYs[2] = {yTop, yTop + numH + rectH + rowGap}; // Oberkante der Nummer

        // Nummerierung: hinten (obere Zeile) 1/2, vorne (untere Zeile) 3/4
        static const uint8_t nums[2][2] = {{1, 2}, {3, 4}};

        display.setTextWrap(false); // langen Namen abschneiden statt umbrechen
        display.setTextSize(2);
        for (int row = 0; row < 2; row++) {
            for (int col = 0; col < 2; col++) {
                const int x = xs[col];
                const int rectY = numYs[row] + numH;
                const uint8_t pos = nums[row][col];
                const AlmemoEinkSensorInfo &s = sensors[pos - 1];

                // Über dem Rechteck: Positionsnummer links, Anzahl Messwerte rechts
                display.setCursor(x, numYs[row]);
                display.print((int)pos);
                if (s.present) {
                    char cnt[8];
                    const int n = snprintf(cnt, sizeof(cnt), "(%u)", s.valueCount);
                    const int cntW = n * 12; // Size 2: 12px/Zeichen -> rechtsbündig
                    display.setCursor(x + rectW - cntW, numYs[row]);
                    display.print(cnt);
                }

                display.drawRect(x, rectY, rectW, rectH, GxEPD_BLACK);

                // Im Rechteck: nur der Sensorname, vertikal zentriert
                display.setCursor(x + 4, rectY + (rectH - 16) / 2);
                display.print(s.present ? s.name : "--");
            }
        }
        display.setTextWrap(true);

        // --- unteres Drittel: Sendeintervall + letzter Timestamp ---
        display.drawFastHLine(0, topH, W, GxEPD_BLACK);
        display.setTextSize(2); // doppelt so groß; "UTC" weggelassen, sonst zu breit (200px)

        char line[32];
        snprintf(line, sizeof(line), "Intervall: %lu s", (unsigned long)intervalSecs);
        display.setCursor(4, topH + 10);
        display.print(line);

        if (lastTimestamp != 0) {
            const unsigned hh = (lastTimestamp / 3600) % 24, mm = (lastTimestamp / 60) % 60, ss = lastTimestamp % 60;
            snprintf(line, sizeof(line), "Letzte: %02u:%02u:%02u", hh, mm, ss);
        } else {
            snprintf(line, sizeof(line), "Letzte: --:--:--");
        }
        display.setCursor(4, topH + 36);
        display.print(line);
    }
};

/**
 * Display mit neuen Daten aktualisieren (zeichnet synchron). Vom AlmemoSenderThread
 * aufzurufen, sobald neue Daten gesendet wurden oder Sensoren an-/abgesteckt werden.
 * sensors[4] = die 4 Rechteck-Positionen (hinten 1/2, vorne 3/4).
 */
void almemoEinkPublish(uint32_t intervalSecs, uint32_t unixTimestamp, const AlmemoEinkSensorInfo sensors[4]);

#endif // ALMEMO_EINK
