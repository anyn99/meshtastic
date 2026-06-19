#include "configuration.h"

#if defined(ALMEMO_EINK)

#include "AlmemoEinkDisplay.h"

void almemoEinkPublish(uint32_t intervalSecs, uint32_t unixTimestamp, const AlmemoEinkSensorInfo sensors[4])
{
    // Funktion-lokales static: wird beim ersten Aufruf zur Laufzeit konstruiert
    // (dann ist der globale SPI bereits initialisiert -> kein static-init-order-Problem).
    static AlmemoEinkDisplay display;
    display.publish(intervalSecs, unixTimestamp, sensors);
}

#endif // ALMEMO_EINK
