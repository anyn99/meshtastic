#include "AlmemoCommon.h"

#include <math.h>

size_t almemoPacketSize(uint8_t count)
{
    return offsetof(AlmemoSensorPacket, values) + (size_t)count * sizeof(AlmemoValue);
}

uint8_t almemoSlotId(uint8_t channel, uint8_t valueIndex)
{
    return (uint8_t)((channel << 4) | (valueIndex & 0x0F));
}

uint8_t almemoSlotChannel(uint8_t slot)
{
    return (uint8_t)(slot >> 4);
}

uint8_t almemoSlotValueIndex(uint8_t slot)
{
    return (uint8_t)(slot & 0x0F);
}

float almemoPow10f(int8_t e)
{
    static const float tbl[] = {1.0f, 10.0f, 100.0f, 1000.0f, 10000.0f};
    if (e >= 0)
        return (e <= 4) ? tbl[e] : 1.0f;
    return (e >= -4) ? (1.0f / tbl[-e]) : 1.0f;
}

float almemoScaleFromRaw(int16_t raw, int8_t exponent)
{
    return (float)raw * almemoPow10f(exponent);
}

int16_t almemoScaleToRaw(float value, int8_t exponent)
{
    float scaled = value * almemoPow10f((int8_t)-exponent);
    if (scaled >= 32767.0f)
        return 32767;
    if (scaled <= -32768.0f)
        return -32768;
    return (int16_t)lroundf(scaled);
}

bool almemoAppendValue(AlmemoSensorPacket &pkt, uint8_t slotId, char u0, char u1, int8_t exponent, int16_t raw)
{
    if (pkt.count >= ALMEMO_MAX_VALUES)
        return false;
    AlmemoValue &v = pkt.values[pkt.count++];
    v.slot     = slotId;
    v.unit[0]  = u0;
    v.unit[1]  = u1;
    v.exponent = exponent;
    v.raw      = raw;
    return true;
}
