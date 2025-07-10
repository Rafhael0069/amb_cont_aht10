#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "include/oled_display.h"

// Configurações do I2C
#define I2C_PORT i2c0
#define PIN_SDA 0 // 2
#define PIN_SCL 1 // 3
#define I2C_BAUDRATE 100000

// Configuração do sensor AHT10
#define AHT10_ADDR 0x38
#define AHT10_CMD_INITIALIZE 0xE1
#define AHT10_CMD_MEASURE 0xAC
#define AHT10_CMD_SOFT_RESET 0xBA
#define AHT10_STATUS_BUSY_MASK 0x80
#define AHT10_STATUS_CAL_MASK 0x08

// Limites para alertas
#define TEMP_ALERT 20.0
#define HUMIDITY_ALERT 70.0


// Variáveis globais
bool alert_condition = false;
absolute_time_t last_alert_time;
const int32_t alert_duration_ms = 5000; // 5 segundos de alerta

// Prototipos de funções
void aht10_init();
void aht10_reset();
bool aht10_read_data(float *humidity, float *temperature);
void check_alert_conditions(float humidity, float temperature);
void display_sensor_data(float humidity, float temperature);

// Inicializa o sensor AHT10
void aht10_init() {
    i2c_init(I2C_PORT, I2C_BAUDRATE);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);

    printf("Inicializando AHT10...\n");
    aht10_reset();

    uint8_t init_cmd[3] = {AHT10_CMD_INITIALIZE, 0x08, 0x00};
    int ret = i2c_write_blocking(I2C_PORT, AHT10_ADDR, init_cmd, 3, false); // Envia comando de inicialização

    if (ret == PICO_ERROR_GENERIC) { // Verifica se houve erro na escrita
        printf("Erro na inicialização do AHT10\n");
        const char *error_msg[] = {"Erro AHT10"};
        oled_display_message(error_msg, 1);
        return;
    }

    sleep_ms(300);

    uint8_t status;
    i2c_read_blocking(I2C_PORT, AHT10_ADDR, &status, 1, false); // Lê o byte de status
    if (!(status & AHT10_STATUS_CAL_MASK)) { // Verifica se o sensor está calibrado
        printf("AHT10 não calibrado!\n");
        const char *cal_msg[] = {"AHT10 Nao Calib.", "Reiniciar sistema"};
        oled_display_message(cal_msg, 2);
    } else { // Se o sensor estiver calibrado
        printf("AHT10 pronto\n");
        const char *ready_msg[] = {"AHT10 OK!"};
        oled_display_message(ready_msg, 1);
    }
}

// Reseta o sensor AHT10
void aht10_reset() {
    uint8_t reset_cmd = AHT10_CMD_SOFT_RESET;
    int ret = i2c_write_blocking(I2C_PORT, AHT10_ADDR, &reset_cmd, 1, false);
    if (ret == PICO_ERROR_GENERIC) {
        printf("Erro no reset do AHT10\n");
    }
    sleep_ms(20);
}

// Lê os dados de umidade e temperatura do AHT10
bool aht10_read_data(float *humidity, float *temperature) {
    uint8_t measure_cmd[3] = {AHT10_CMD_MEASURE, 0x33, 0x00};
    int ret = i2c_write_blocking(I2C_PORT, AHT10_ADDR, measure_cmd, 3, false);
    if (ret == PICO_ERROR_GENERIC) { // Verifica se houve erro na escrita
        printf("Erro no comando de medição\n");
        return false;
    }

    sleep_ms(80);

    uint8_t status_byte;
    i2c_read_blocking(I2C_PORT, AHT10_ADDR, &status_byte, 1, false);

    if (status_byte & AHT10_STATUS_BUSY_MASK) { // Verifica se o sensor está ocupado
        printf("AHT10 ocupado\n");
        return false;
    }

    uint8_t data[6];
    ret = i2c_read_blocking(I2C_PORT, AHT10_ADDR, data, 6, false);
    if (ret == PICO_ERROR_GENERIC) { // Verifica se houve erro na leitura
        printf("Erro na leitura de dados\n");
        return false;
    }

    uint32_t raw_humidity = ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3];
    raw_humidity = raw_humidity >> 4;

    uint32_t raw_temperature = ((uint32_t)data[3] & 0x0F) << 16 | ((uint32_t)data[4] << 8) | data[5];

    *humidity = (float)raw_humidity * 100.0f / 1048576.0f;
    *temperature = (float)raw_temperature * 200.0f / 1048576.0f - 50.0f;

    return true;
}

void check_alert_conditions(float humidity, float temperature) {
    bool new_alert = (humidity > HUMIDITY_ALERT) || 
                    (temperature < TEMP_ALERT);
    
    if (new_alert) {
        alert_condition = true;
        last_alert_time = get_absolute_time();
    } else if (alert_condition && 
               absolute_time_diff_us(last_alert_time, get_absolute_time()) > alert_duration_ms * 1000) {
        alert_condition = false;
    }
}

void display_sensor_data(float humidity, float temperature) {
    char temp_str[20];
    char hum_str[20];
    char alert_str[20] = "";
    
    snprintf(temp_str, sizeof(temp_str), "Temp: %.1f C", temperature);
    snprintf(hum_str, sizeof(hum_str), "Umid: %.1f %%", humidity);
    
    if (alert_condition) {
        if (humidity > HUMIDITY_ALERT) {
            snprintf(alert_str, sizeof(alert_str), "ALTO: Umidade!");
        } else if (temperature < TEMP_ALERT) {
            snprintf(alert_str, sizeof(alert_str), "BAIXA: Temp!");
        }
        
        const char *alert_msgs[] = {temp_str, hum_str, "", alert_str};
        oled_display_message(alert_msgs, 4);
    } else {
        const char *normal_msgs[] = {temp_str, hum_str, "Ambiente OK"};
        oled_display_message(normal_msgs, 3);
    }
}

int main() {
    stdio_init_all();
    
    // Inicializa OLED
    oled_init();
    const char *init_msg[] = {"Sistema de", "Monitoramento", "Ambiental"};
    oled_display_message(init_msg, 3);
    sleep_ms(5000); // Exibe mensagem de inicialização por 5 segundos
    
    // Inicializa AHT10
    aht10_init();
    sleep_ms(1000);
    
    float humidity, temperature;
    
    while (true) {
        if (aht10_read_data(&humidity, &temperature)) {
            printf("Umidade: %.2f%%, Temperatura: %.2fC\n", humidity, temperature);
            
            check_alert_conditions(humidity, temperature);
            display_sensor_data(humidity, temperature);
        } else {
            printf("Falha na leitura\n");
            const char *error_msg[] = {"Erro na leitura", "do sensor AHT10"};
            oled_display_message(error_msg, 2);
            aht10_reset();
        }

        sleep_ms(2000); // Atualiza a cada 2 segundos
    }
    
    return 0;
}