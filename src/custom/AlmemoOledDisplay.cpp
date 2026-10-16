#include "configuration.h"

#if defined(ALMEMO_OLED)

#include "AlmemoOledDisplay.h"

// In main.cpp angelegt (wie almemoLedThread), damit der Konstruktor erst laeuft,
// wenn Wire bereits initialisiert ist -> kein static-init-order-Problem.
AlmemoOledThread *almemoOledThread;

void almemoOledPublish(const AlmemoSensorPacket &pkt, const AlmemoMenuSensorInfo &info, uint32_t sendCount,
                       uint32_t intervalSecs)
{
    if (almemoOledThread)
        almemoOledThread->publish(pkt, info, sendCount, intervalSecs);
}

void almemoOledClear()
{
    if (almemoOledThread)
        almemoOledThread->clear();
}

#endif // ALMEMO_OLED
