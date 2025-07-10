#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <stdint.h>
#include "inc/ssd1306.h"

// Função genérica para exibir mensagens no OLED.
void oled_display_message(const char *messages[], size_t line_count);

void oled_init();
void oled_clear();

#endif // OLED_DISPLAY_H