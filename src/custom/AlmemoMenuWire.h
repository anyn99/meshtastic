#pragma once

#include <stdint.h>

/*
 * AlmemoMenuWire — das I2C-Format des Menue-Kanals.
 *
 * ABSICHTLICH ABHAENGIGKEITSFREI: nur <stdint.h>. Diesen Header ziehen BEIDE
 * Seiten, und der Receiver darf AlmemoMenu.h zwar inzwischen einbinden (das
 * Modell ist seit dem Renderer-Split displayfrei), soll aber fuer das reine
 * Drahtformat nicht darauf angewiesen sein.
 *
 * Der Receiver bedient dafuer eine eigene Slave-Adresse, zusaetzlich zu den
 * beiden EEPROM-Fenstern (0x50/0x51) und dem Register-Device (0x40). Die
 * ALMEMO-Emulation bleibt davon unberuehrt.
 *
 * ---------------------------------------------------------------------------
 * ADRESSIERUNG: 16 Bit, big endian, wie bei den groesseren I2C-EEPROMs
 * (24C32 und aufwaerts). Der Master schreibt zwei Bytes -- erst hi, dann lo --
 * und liest danach ab dieser Stelle.
 *
 * Grund: eine rohe MenuStruct ist rund 1,5 kB. Mit einem Byte Zeiger waeren
 * nur 256 Byte adressierbar; die erste Fassung dieses Headers hatte genau da
 * ein Loch. Es hing noch nichts davon ab, deshalb korrigiert statt umgangen.
 *
 *   0x0000   Kopf, 20 B (MenuWireHeader)
 *   0x0020   MenuStruct, roh, structSize Byte
 *   0x0800   Werteblock, valueCount * sizeof(MenuValue)
 *   0x1000   Edit (nur schreibbar), s. ALMEMO_MENU_OFF_EDIT
 *
 * Pro Transaktion gehen maximal ALMEMO_MENU_CHUNK Byte -- begrenzt von beiden
 * Seiten: SERIAL_BUFFER_SIZE (Wire-RingBuffer des Nodes) und I2C_BB_MAX_BUF
 * (Sendepuffer des Bitbang-Slaves) sind beide 64. Der Node holt die Struktur
 * also in 24 Bloecken, zusammen rund 295 ms bei 48,8 kHz.
 *
 * Deshalb pollt der Node NUR den Kopf und vergleicht `hash`; die Struktur holt
 * er ausschliesslich, wenn sie sich geaendert hat. Im Normalbetrieb wandert nur
 * der Werteblock -- 320 Byte, rund 60 ms.
 * ---------------------------------------------------------------------------
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
 * dafuer ist der Hash da). Entspricht MenuStruct::modelVersion.
 *
 * Solange nichts ausgeliefert ist, bleibt es bei 1 und das Format wird einfach
 * angepasst -- Node und Receiver werden ohnehin zusammen geflasht. Ab der
 * ersten Auslieferung gilt: jede Formataenderung zaehlt hoch. */
#define ALMEMO_MENU_MODEL_VERSION 1

/* Fensteraufteilung, vom Master als 16-Bit-Zeiger geschrieben. */
#define ALMEMO_MENU_OFF_HEADER 0x0000u
#define ALMEMO_MENU_OFF_STRUCT 0x0020u
#define ALMEMO_MENU_OFF_VALUES 0x0800u

/* Maximale Nutzlast pro Lese-Transaktion, s. o. */
#define ALMEMO_MENU_CHUNK 64u

/*
 * Edit-Fenster: NUR schreibbar, Lesen dort liefert nichts.
 *
 * Eine Aenderung ist ein Write von 7 Byte:
 *
 *   [0..1]  Zeiger = ALMEMO_MENU_OFF_EDIT, big endian (wie jeder Zugriff)
 *   [2]     Item-Index im Menue des Eigentuemers
 *   [3..6]  neuer Rohwert, int32 little endian
 *
 * Der Item-Index genuegt, weil der Eigentuemer aus seinem eigenen
 * MenuItem::setting die Bedeutung ableitet. Das `targets`-Byte aus MenuEdit
 * wandert NICHT mit: der Host wendet seinen Anteil selbst an und schickt nur
 * den Anteil des Eigentuemers.
 *
 * Laenge und Zeiger zusammen machen den Edit von einem reinen Zeiger-Write (2
 * Byte) unterscheidbar -- der Slave braucht also keinen Modus.
 */
#define ALMEMO_MENU_OFF_EDIT 0x1000u
#define ALMEMO_MENU_EDIT_LEN 7u

/**
 * Kopf des Menue-Kanals, 20 Byte, little endian, ohne implizites Padding.
 *
 * Die Zaehler stehen hier, damit der Node die Uebertragung dimensionieren und
 * gegen seine eigenen MENU_MAX_* pruefen kann, ohne die Struktur erst zu holen.
 *
 * `structSize` ist die `sizeof(MenuStruct)` der Gegenseite. Die Struktur geht
 * ROH ueber den Bus, und beide Boards werden getrennt geflasht: stimmen die
 * Zaehler, aber die MENU_MAX_*-Kapazitaeten unterscheiden sich, liegen die
 * Array-Offsets verschoben und man liest lautlos Muell. Der Vergleich mit der
 * eigenen sizeof faengt genau das ab.
 *
 * `hash` ist der FNV-1a aus MenuStruct und entscheidet, ob ueberhaupt geholt
 * werden muss.
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
    uint16_t structSize;
    uint16_t _pad2;
    uint32_t hash; /* 4-Byte-ausgerichtet auf Offset 16 */
};

static_assert(sizeof(MenuWireHeader) == 20, "MenuWireHeader muss 20 Byte sein");
