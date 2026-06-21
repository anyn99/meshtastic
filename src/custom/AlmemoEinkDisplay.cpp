#include "configuration.h"

#if defined(ALMEMO_EINK)

#include "AlmemoEinkDisplay.h"

// Funktion-lokales static: wird beim ERSTEN Aufruf zur Laufzeit konstruiert (dann ist der
// globale SPI bereits initialisiert -> kein static-init-order-Problem). Der Konstruktor
// fährt das Display einmalig hoch und zeichnet den leeren Startbildschirm.
static AlmemoEinkDisplay &einkDisplay()
{
    static AlmemoEinkDisplay display;
    return display;
}

void almemoEinkInitBlank()
{
    einkDisplay(); // erzwingt die Konstruktion -> Display kommt leer hoch
}

void almemoEinkPublish(uint32_t intervalSecs, uint32_t sendCount, const AlmemoEinkSensorInfo sensors[4])
{
    einkDisplay().publish(intervalSecs, sendCount, sensors);
}

#endif // ALMEMO_EINK
