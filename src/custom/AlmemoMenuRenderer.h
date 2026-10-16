#pragma once

#include "AlmemoMenu.h"

#if defined(ALMEMO_OLED)

#include <SSD1306Wire.h>

/*
 * AlmemoMenuRenderer — zeichnet eine MenuStruct auf ein SSD1306.
 *
 * Abgetrennt von AlmemoMenu.h, weil DAS MODELL geraeteunabhaengig sein muss:
 * der Receiver baut seine Menuestruktur mit demselben MenuBuilder, hat aber
 * kein Display und darf <SSD1306Wire.h> nicht sehen. Hier steht also alles,
 * was die Display-Bibliothek braucht -- und nur das.
 */

// =========================================================================
//  Renderer
// =========================================================================

class MenuRenderer
{
  public:
    static constexpr int16_t HEADER_Y = 0;
    static constexpr int16_t RULE_Y   = 12;
    static constexpr int16_t ROW_Y0   = 14;
    static constexpr int16_t ROW_DY   = 10;
    static constexpr int16_t WIDTH    = 128;

    static void render(SSD1306Wire &d, const MenuStruct &s, const MenuValue *vals, const MenuState &st)
    {
        d.clear();
        switch (st.mode) {
        case MenuMode::Carousel:
            drawStatus(d, s, vals, st);
            break;
        case MenuMode::Menu:
            drawMenu(d, s, st);
            break;
        case MenuMode::SelectList:
            drawSelect(d, s, vals, st);
            break;
        case MenuMode::DigitEdit:
            drawDigits(d, s, vals, st);
            break;
        case MenuMode::Confirm:
            drawConfirm(d, s, vals, st);
            break;
        case MenuMode::Invalid:
            drawInvalid(d, s, vals, st);
            break;
        }
        d.display();
    }

    /** Bildschirm fuer "noch keine Struktur da". */
    static void renderWaiting(SSD1306Wire &d, const char *text)
    {
        d.clear();
        d.setTextAlignment(TEXT_ALIGN_CENTER);
        d.drawString(WIDTH / 2, 28, text);
        d.display();
    }

  private:
    static int16_t rowY(uint8_t row) { return (int16_t)(ROW_Y0 + row * ROW_DY); }

    static void header(SSD1306Wire &d, const char *left, const char *right)
    {
        d.setTextAlignment(TEXT_ALIGN_LEFT);
        if (left && left[0])
            d.drawString(0, HEADER_Y, left);
        if (right && right[0]) {
            d.setTextAlignment(TEXT_ALIGN_RIGHT);
            d.drawString(WIDTH - 1, HEADER_Y, right);
        }
        d.drawHorizontalLine(0, RULE_Y, WIDTH);
    }

    /** Label links, Wert (+Einheit) rechts. */
    static void labelValue(SSD1306Wire &d, uint8_t row, const char *label, const char *value)
    {
        d.setTextAlignment(TEXT_ALIGN_LEFT);
        d.drawString(0, rowY(row), label);
        if (value && value[0]) {
            d.setTextAlignment(TEXT_ALIGN_RIGHT);
            d.drawString(WIDTH - 1, rowY(row), value);
        }
    }

    static void drawStatus(SSD1306Wire &d, const MenuStruct &s, const MenuValue *vals, const MenuState &st)
    {
        const MenuPage &p = s.pages[st.statusPage];
        header(d, s.text(s.deviceName), s.text(p.title));

        for (uint8_t r = 0; r < p.itemCount && r < MENU_ROWS; r++) {
            const MenuItem &it = s.items[p.firstItem + r];

            if ((MenuItemKind)it.kind == MenuItemKind::Hint) {
                d.setTextAlignment(TEXT_ALIGN_LEFT);
                d.drawString(0, rowY(r), s.text(it.label));
                continue;
            }

            char val[24] = {0};
            if (it.value != MENU_NO_IDX && vals) {
                const MenuValueKind vk = (MenuValueKind)vals[it.value].kind;
                const bool numeric = (vk == MenuValueKind::Int || vk == MenuValueKind::Fixed);
                menufmt::value(s, vals[it.value], val, sizeof(val));
                /* Einheit nur bei Zahlen — sonst stuende "USB versorgt mV" da. */
                if (numeric && it.unit != MENU_NO_TEXT && val[0]) {
                    char u[6];
                    menufmt::unit(s.text(it.unit), u, sizeof(u));
                    if (u[0]) {
                        strncat(val, " ", sizeof(val) - strlen(val) - 1);
                        strncat(val, u, sizeof(val) - strlen(val) - 1);
                    }
                }
            }
            labelValue(d, r, s.text(it.label), val);
        }
    }

    /** Liste mit Cursor; letzte Zeile Tastenhinweis. */
    static void drawList(SSD1306Wire &d, const char *const *labels, uint8_t count, uint8_t cursor, uint8_t scroll,
                         const char *hint)
    {
        const uint8_t window = (uint8_t)(MENU_ROWS - (hint ? 1 : 0));
        d.setTextAlignment(TEXT_ALIGN_LEFT);
        for (uint8_t r = 0; r < window; r++) {
            const uint8_t i = (uint8_t)(scroll + r);
            if (i >= count)
                break;
            char line[30];
            snprintf(line, sizeof(line), "%s %s", i == cursor ? ">" : " ", labels[i]);
            d.drawString(0, rowY(r), line);
        }
        if (hint)
            d.drawString(0, rowY(MENU_ROWS - 1), hint);
    }

    static void drawMenu(SSD1306Wire &d, const MenuStruct &s, const MenuState &st)
    {
        const MenuPage &p = s.pages[st.menuPage];
        header(d, s.text(p.title), nullptr);

        const char *labels[MENU_MAX_ITEMS];
        for (uint8_t i = 0; i < p.itemCount && i < MENU_MAX_ITEMS; i++)
            labels[i] = s.text(s.items[p.firstItem + i].label);

        drawList(d, labels, p.itemCount, st.cursor, st.scroll, "kurz=weiter  lang=OK");
    }

    /** Kopfzeile der Edit-Screens: Titel links, aktueller Wert rechts. */
    static void editHeader(SSD1306Wire &d, const MenuStruct &s, const MenuValue *vals, const MenuState &st)
    {
        const MenuItem &it = s.items[st.editItem];
        char cur[24]       = {0};
        if (it.value != MENU_NO_IDX && vals) {
            char v[20];
            menufmt::value(s, vals[it.value], v, sizeof(v));
            snprintf(cur, sizeof(cur), "akt. %s", v);
        }
        header(d, s.text(it.label), cur);
    }

    static void drawSelect(SSD1306Wire &d, const MenuStruct &s, const MenuValue *vals, const MenuState &st)
    {
        editHeader(d, s, vals, st);
        const MenuEnum &e = s.enums[st.editEnum];

        const char *labels[MENU_MAX_OPTIONS];
        for (uint8_t i = 0; i < e.count && i < MENU_MAX_OPTIONS; i++)
            labels[i] = s.text(s.options[e.firstOption + i].label);

        drawList(d, labels, e.count, st.cursor, st.scroll, "kurz=weiter  lang=OK");
    }

    static void drawDigits(SSD1306Wire &d, const MenuStruct &s, const MenuValue *vals, const MenuState &st)
    {
        editHeader(d, s, vals, st);

        /* Ziffern mit Abstand, aktive Stelle unterstrichen. */
        const int16_t x0   = 0;
        const int16_t step = 14;
        d.setTextAlignment(TEXT_ALIGN_LEFT);
        for (uint8_t i = 0; i < st.digitCount; i++) {
            char c[2] = {(char)('0' + st.digits[i]), 0};
            const int16_t x = (int16_t)(x0 + i * step);
            d.drawString(x, rowY(0), c);
            if (i == st.activeDigit)
                d.drawHorizontalLine(x, (int16_t)(rowY(0) + 11), 8);
        }

        int32_t v = 0;
        for (uint8_t i = 0; i < st.digitCount; i++)
            v = v * 10 + st.digits[i];
        char sum[28];
        char dur[20];
        menufmt::duration(v, dur, sizeof(dur));
        snprintf(sum, sizeof(sum), "= %s", dur);
        d.drawString(0, rowY(1), sum);

        d.drawString(0, rowY(MENU_ROWS - 1), "kurz=+1  lang=weiter");
    }

    static void drawActions(SSD1306Wire &d, const MenuState &st, uint8_t firstRow, const char *hint)
    {
        const char *labels[4];
        for (uint8_t i = 0; i < st.actionCount; i++) {
            switch (st.actions[i]) {
            case MenuAction::Apply:
                labels[i] = "Übernehmen";
                break;
            case MenuAction::ApplyBoth:
                labels[i] = "Auf beide anwenden";
                break;
            case MenuAction::Correct:
                labels[i] = "Korrigieren";
                break;
            default:
                labels[i] = "Abbrechen";
                break;
            }
        }
        d.setTextAlignment(TEXT_ALIGN_LEFT);
        for (uint8_t i = 0; i < st.actionCount && firstRow + i < MENU_ROWS; i++) {
            char line[30];
            snprintf(line, sizeof(line), "%s %s", i == st.actionCursor ? ">" : " ", labels[i]);
            d.drawString(0, rowY((uint8_t)(firstRow + i)), line);
        }
        if (hint && firstRow + st.actionCount < MENU_ROWS)
            d.drawString(0, rowY(MENU_ROWS - 1), hint);
    }

    static void drawConfirm(SSD1306Wire &d, const MenuStruct &s, const MenuValue *vals, const MenuState &st)
    {
        if (st.offerBoth)
            header(d, "RECEIVER ANPASSEN!", nullptr);
        else
            editHeader(d, s, vals, st);

        /* Zeile 1: der neue Wert, bei Dauern mit Klartext in Klammern. */
        char neu[30];
        const MenuItem &it = s.items[st.editItem];
        if (it.value != MENU_NO_IDX && vals && (MenuValueKind)vals[it.value].kind == MenuValueKind::Duration) {
            char dur[20];
            menufmt::duration(st.candidate, dur, sizeof(dur));
            if (st.candidate >= 60)
                snprintf(neu, sizeof(neu), "neu: %lds (%s)", (long)st.candidate, dur);
            else
                snprintf(neu, sizeof(neu), "neu: %s", dur);
        } else {
            const char *txt = optionLabel(s, st, st.candidate);
            if (txt)
                snprintf(neu, sizeof(neu), "neu: %s", txt);
            else
                snprintf(neu, sizeof(neu), "neu: %ld", (long)st.candidate);
        }
        d.setTextAlignment(TEXT_ALIGN_LEFT);
        d.drawString(0, rowY(0), neu);

        drawActions(d, st, 1, "kurz=weiter  lang=OK");
    }

    static void drawInvalid(SSD1306Wire &d, const MenuStruct &s, const MenuValue *vals, const MenuState &st)
    {
        editHeader(d, s, vals, st);
        d.setTextAlignment(TEXT_ALIGN_LEFT);
        d.drawString(0, rowY(0), st.message);
        drawActions(d, st, 1, "kurz=weiter  lang=OK");
    }

    /** Text einer Option zu einem Rohwert, falls die Einstellung eine Liste hat. */
    static const char *optionLabel(const MenuStruct &s, const MenuState &st, int32_t raw)
    {
        if (st.editEnum == MENU_NO_IDX)
            return nullptr;
        const MenuEnum &e = s.enums[st.editEnum];
        for (uint8_t i = 0; i < e.count; i++)
            if (s.options[e.firstOption + i].raw == raw)
                return s.text(s.options[e.firstOption + i].label);
        return nullptr;
    }
};

#endif // ALMEMO_OLED
