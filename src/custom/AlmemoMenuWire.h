#pragma once

#include <stdint.h>

/*
 * AlmemoMenuWire — das I2C-Format des Menue-Kanals.
 *
 * ABSICHTLICH ABHAENGIGKEITSFREI: nur <stdint.h>. Diesen Header ziehen BEIDE
 * Seiten, und der Receiver darf AlmemoMenu.h nicht einbinden -- das zieht
 * <SSD1306Wire.h> nach, und auf dem Receiver gibt es kein Display.
 *
 * Der Receiver bedient dafuer eine eigene Slave-Adresse, zusaetzlich zu den
 * beiden EEPROM-Fenstern (0x50/0x51) und dem Register-Device (0x40). Die
 * ALMEMO-Emulation bleibt davon unberuehrt.
 *
 *   Offset 0x00   Kopf, 16 B (MenuWireHeader)
 *   Offset 0x10   Struktur in Bloecken
 *   Offset 0x80   Werteblock
 *   Write         Adresszeiger (1 Byte), spaeter Edit: itemIdx + raw
 *
 * Der Node pollt NUR den Kopf und vergleicht `hash`; die Struktur holt er
 * ausschliesslich, wenn sie sich geaendert hat. Im Normalbetrieb wandert also
 * nur der Werteblock ueber den Bus -- bei 48,8 kHz rund 50 ms.
 *
 * Zweiter Zweck des Kopfes: findet der Node hier ein gueltiges `magic`, dann
 * steckt ein Receiver und kein ALMEMO-Fuehler. Diese Andock-Kennung soll
 * spaeter das Sensor-Probing auf 0x50/0x51 unterdruecken, sonst gibt es eine
 * Rueckkopplung -- der Receiver gibt sich ja absichtlich als Fuehler aus.
 */

/* Adresse des Menue-Kanals.
 *
 * Hinweis: 0x41 ist in Meshtastics eigener Adresstabelle INA_ADDR_ALTERNATE.
 * Fuer unseren Node ist das folgenlos, weil sein Boot-Scan abgeschaltet ist
 * (s. main.cpp); ein fremder Meshtastic-Node wuerde den angedockten Receiver
 * hier aber als INA219 erkennen -- genau wie heute schon ueber 0x40. */
#ifndef ALMEMO_MENU_I2C_ADDR
#define ALMEMO_MENU_I2C_ADDR 0x41
#endif

/* Little endian auf dem Draht -> Bytes 0x4D 0x41 = "MA", im Dump erkennbar. */
#define ALMEMO_MENU_MAGIC 0x414D

/* Hochzaehlen, wenn sich DIESES Drahtformat aendert (nicht bei Menueinhalt --
 * dafuer ist der Hash da). Entspricht MenuStruct::modelVersion. */
#define ALMEMO_MENU_MODEL_VERSION 1

/* Fensteraufteilung, vom Master als Adresszeiger geschrieben. */
#define ALMEMO_MENU_OFF_HEADER 0x00
#define ALMEMO_MENU_OFF_STRUCT 0x10
#define ALMEMO_MENU_OFF_VALUES 0x80

/**
 * Kopf des Menue-Kanals, 16 Byte, little endian.
 *
 * Die Zaehler stehen hier, damit der Node die Struktur-Uebertragung dimensio-
 * nieren kann, ohne sie erst zu holen. `hash` ist der FNV-1a aus MenuStruct und
 * entscheidet, ob ueberhaupt geholt werden muss.
 */
struct MenuWireHeader {
    uint16_t magic; /* ALMEMO_MENU_MAGIC */
    uint8_t  modelVersion;
    uint8_t  pageCount;
    uint8_t  itemCount;
    uint8_t  valueCount;
    uint8_t  enumCount;
    uint8_t  optionCount;
    uint8_t  numberCount;
    uint8_t  _pad;
    uint16_t poolLen;
    uint32_t hash;
};

static_assert(sizeof(MenuWireHeader) == 16, "MenuWireHeader muss 16 Byte sein");
