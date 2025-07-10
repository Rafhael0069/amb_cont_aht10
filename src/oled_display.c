#include <string.h>
#include "include/oled_display.h"

// Buffer e área de renderização.
static uint8_t ssd[ssd1306_buffer_length];  // Buffer para o display OLED.
static struct render_area frame_area;       // Área de renderização.

/**
 * Inicializa o display OLED.
 */
void oled_init() {
    // Inicializa o I2C e o display SSD1306.
    i2c_init(i2c1, ssd1306_i2c_clock * 1000);  // Configura o I2C com o clock especificado.
    gpio_set_function(14, GPIO_FUNC_I2C);      // Configura o pino 14 como I2C.
    gpio_set_function(15, GPIO_FUNC_I2C);      // Configura o pino 15 como I2C.
    gpio_pull_up(14);                          // Habilita pull-up no pino 14.
    gpio_pull_up(15);                          // Habilita pull-up no pino 15.

    ssd1306_init();  // Inicializa o display SSD1306.

    // Configura a área de renderização.
    frame_area.start_column = 0;
    frame_area.end_column = ssd1306_width - 1;
    frame_area.start_page = 0;
    frame_area.end_page = ssd1306_n_pages - 1;
    calculate_render_area_buffer_length(&frame_area);

    oled_clear();  // Limpa o display.
}

/**
 * Limpa o conteúdo do display.
 */
void oled_clear() {
    memset(ssd, 0, ssd1306_buffer_length);  // Preenche o buffer com zeros.
    render_on_display(ssd, &frame_area);     // Renderiza o buffer no display.
}

/**
 * Exibe uma mensagem no display OLED.
 * @param messages: Array de strings contendo as mensagens.
 * @param line_count: Número de linhas de mensagens.
 */
void oled_display_message(const char *messages[], size_t line_count) {
    oled_clear();  // Limpa o display antes de exibir a nova mensagem.

    // Calcula o deslocamento vertical para centralizar as mensagens.
    int total_height = line_count * 8;  // Cada linha tem 8 pixels de altura.
    int start_y = (64 - total_height) / 2;

    for (size_t i = 0; i < line_count; i++) {
        int message_length = strlen(messages[i]);
        int start_x = (128 - (message_length * 6)) / 2;  // Cada caractere tem 6 pixels de largura.

        // Exibe cada linha no display.
        ssd1306_draw_string(ssd, start_x, start_y + (i * 8), messages[i]);
    }

    render_on_display(ssd, &frame_area);  // Renderiza o buffer no display.
}