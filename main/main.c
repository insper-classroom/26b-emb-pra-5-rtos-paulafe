/*
 * LAB 5 - RTOS (Prática)
 * Sensor ultrassônico HC-SR04 com FreeRTOS, resultado no OLED e no LED RGB.
 *
 * Estrutura (conforme o diagrama do enunciado):
 *
 *   pin_callback --xQueueTime--> echo_task --xQueueDistance--> oled_task
 *   trigger_task --xSemaphoreTrigger-----------------------> oled_task
 *   trigger_task --pulso de 10 us--> HC-SR04 --echo--> pin_callback
 *
 * Regra 4.4: não há variável global além dos recursos do RTOS.
 */
#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <stdio.h>
#include <task.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pins.h"
#include "ssd1306.h"

// === Pinos do HC-SR04 ===
// GP14 e GP16 são do conector do LCD, livres quando o LCD não é usado.
// const int TRIG_PIN = 14;
// const int ECHO_PIN = 16;

// === Constantes do sensor ===
const int TRIGGER_PULSE_US = 10;    // largura do pulso de trigger (datasheet)
const int TRIGGER_PERIOD_MS = 300;  // intervalo entre leituras (>= 60 ms)
const int US_POR_CM = 58;           // som ida e volta: ~58 us por cm
const int DIST_MAX_CM = 400;        // alcance máximo do HC-SR04

// Maior pulso de echo válido: 400 cm * 58 us = 23,2 ms. Arredondado para cima.
const int ECHO_FALL_TIMEOUT_MS = 30;

// Tempo que a oled_task espera pela distância depois do trigger.
// Precisa cobrir o maior echo válido (~23 ms) e ser menor que TRIGGER_PERIOD_MS.
const int DISTANCE_TIMEOUT_MS = 50;

// === Constantes de display e LED ===
const int OLED_I2C_FREQ_HZ = 400000;
const int OLED_SDA_PIN = 2;
const int OLED_SCL_PIN = 3;
const int OLED_ADDR = 0x3C;
const int OLED_WIDTH = 128;
const int OLED_HEIGHT = 64;
const int BAR_Y = 28;
const int BAR_HEIGHT = 4;
const int LED_LIMITE_CM = 100;      // até 1 m: verde; acima: amarelo

// === Tamanhos das filas e stacks ===
const int TIME_QUEUE_LEN = 4;
const int DISTANCE_QUEUE_LEN = 4;
const int TASK_STACK_SIZE = 4096;
const int TASK_PRIORITY = 1;

// === Recursos do RTOS (únicos globais permitidos pela regra 4.4) ===
static QueueHandle_t xQueueTime = NULL;
static QueueHandle_t xQueueDistance = NULL;
static SemaphoreHandle_t xSemaphoreTrigger = NULL;

// === Callback do pino do echo (ISR) ===
// Só anota o instante da borda e envia pela fila. Regra 4.1: API FromISR.
void pin_callback(uint gpio, uint32_t events) {
    if (gpio != ECHO_PIN) {
        return;
    }

    if (events & (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL)) {
        uint64_t t = to_us_since_boot(get_absolute_time());
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(xQueueTime, &t, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// === LED RGB (ativo em nível baixo: 0 acende, 1 apaga) ===
static void led_rgb_init(void) {
    gpio_init(LED_PIN_R);
    gpio_set_dir(LED_PIN_R, GPIO_OUT);
    gpio_init(LED_PIN_G);
    gpio_set_dir(LED_PIN_G, GPIO_OUT);
    gpio_init(LED_PIN_B);
    gpio_set_dir(LED_PIN_B, GPIO_OUT);
}

static void led_rgb_set(bool r, bool g, bool b) {
    gpio_put(LED_PIN_R, !r);
    gpio_put(LED_PIN_G, !g);
    gpio_put(LED_PIN_B, !b);
}

// === OLED ===
static void oled_display_init(ssd1306_t *disp) {
    i2c_init(i2c1, OLED_I2C_FREQ_HZ);
    gpio_set_function(OLED_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SDA_PIN);
    gpio_pull_up(OLED_SCL_PIN);

    disp->external_vcc = false;
    ssd1306_init(disp, OLED_WIDTH, OLED_HEIGHT, OLED_ADDR, i2c1);
    ssd1306_clear(disp);
    ssd1306_show(disp);

    gpio_init(SSD1306_PIN_LITE);
    gpio_set_dir(SSD1306_PIN_LITE, GPIO_OUT);
    gpio_put(SSD1306_PIN_LITE, 0);
}

static void oled_show_distance(ssd1306_t *disp, int dist_cm) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%d cm", dist_cm);

    // Barra proporcional à distância (0 a DIST_MAX_CM -> 0 a OLED_WIDTH)
    uint32_t bar_w = (uint32_t)(dist_cm * OLED_WIDTH / DIST_MAX_CM);
    if (bar_w > (uint32_t)OLED_WIDTH) {
        bar_w = (uint32_t)OLED_WIDTH;
    }

    ssd1306_clear(disp);
    ssd1306_draw_string(disp, 0, 0, 1, "Distancia:");
    ssd1306_draw_string(disp, 0, 10, 2, buf);
    if (bar_w > 0) {
        ssd1306_draw_square(disp, 0, BAR_Y, bar_w, BAR_HEIGHT);
    }
    ssd1306_show(disp);
}

static void oled_show_failure(ssd1306_t *disp) {
    ssd1306_clear(disp);
    ssd1306_draw_string(disp, 0, 0, 1, "Sensor:");
    ssd1306_draw_string(disp, 0, 10, 2, "FALHA");
    ssd1306_show(disp);
}

// === trigger_task: gera o pulso e avisa o OLED ===
void trigger_task(void *p) {
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);

    while (1) {
        gpio_put(TRIG_PIN, 1);
        sleep_us(TRIGGER_PULSE_US); // regra 4.3: exceção para delay em us
        gpio_put(TRIG_PIN, 0);

        xSemaphoreGive(xSemaphoreTrigger);

        vTaskDelay(pdMS_TO_TICKS(TRIGGER_PERIOD_MS));
    }
}

// === echo_task: transforma os dois instantes em distância ===
void echo_task(void *p) {
    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_pull_down(ECHO_PIN); // evita bordas espúrias com o sensor desconectado

    // Habilitada aqui: o callback é atendido pelo core que configurou a IRQ.
    gpio_set_irq_enabled_with_callback(ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true,
                                       &pin_callback);

    uint64_t t_rise;
    uint64_t t_fall;

    while (1) {
        // Borda de subida: espera indefinida, o trigger é periódico.
        if (xQueueReceive(xQueueTime, &t_rise, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        // Borda de descida: se não vier a tempo, descarta e ressincroniza.
        if (xQueueReceive(xQueueTime, &t_fall, pdMS_TO_TICKS(ECHO_FALL_TIMEOUT_MS)) != pdTRUE) {
            xQueueReset(xQueueTime);
            continue;
        }

        int dist_cm = (int)((t_fall - t_rise) / US_POR_CM);

        // Leitura fora do alcance não é enviada: a oled_task cai no timeout
        // e trata como falha.
        if (dist_cm <= DIST_MAX_CM) {
            xQueueSend(xQueueDistance, &dist_cm, 0);
        }
    }
}

// === oled_task: exibe distância/falha no OLED e no LED ===
void oled_task(void *p) {
    ssd1306_t disp; // local: regra 4.4
    oled_display_init(&disp);
    led_rgb_init();
    led_rgb_set(false, false, false);

    int dist_cm;

    while (1) {
        // Espera uma leitura ser disparada.
        if (xSemaphoreTake(xSemaphoreTrigger, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (xQueueReceive(xQueueDistance, &dist_cm, pdMS_TO_TICKS(DISTANCE_TIMEOUT_MS)) == pdTRUE) {
            // Dado chegou no tempo certo.
            oled_show_distance(&disp, dist_cm);
            if (dist_cm <= LED_LIMITE_CM) {
                led_rgb_set(false, true, false); // verde
            }
            else {
                led_rgb_set(true, true, false); // amarelo = vermelho + verde
            }
        }
        else {
            // Dado não chegou: falha no sensor.
            oled_show_failure(&disp);
            led_rgb_set(true, false, false); // vermelho
        }
    }
}

int main() {
    stdio_init_all();

    xQueueTime = xQueueCreate(TIME_QUEUE_LEN, sizeof(uint64_t));
    xQueueDistance = xQueueCreate(DISTANCE_QUEUE_LEN, sizeof(int));
    xSemaphoreTrigger = xSemaphoreCreateBinary();

    if (xQueueTime == NULL || xQueueDistance == NULL || xSemaphoreTrigger == NULL) {
        while (1) {
            // falha ao criar recursos do RTOS
        }
    }

    xTaskCreate(trigger_task, "Trigger", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL);
    xTaskCreate(echo_task, "Echo", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL);
    xTaskCreate(oled_task, "OLED", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL);

    vTaskStartScheduler();

    while (true) {
    }
}
// /*
//  * LED blink with FreeRTOS
//  */
// #include <FreeRTOS.h>
// #include <queue.h>
// #include <semphr.h>
// #include <stdio.h>
// #include <task.h>

// #include "hardware/gpio.h"
// #include "hardware/i2c.h"
// #include "pico/stdlib.h"
// #include "pins.h"
// #include "ssd1306.h"

// // === Definições para SSD1306 ===
// ssd1306_t disp;

// #define BTN_QUEUE_LEN 8

// static QueueHandle_t xBtnQueue = NULL;

// // === Função de callback (IRQ) ===
// void btn_callback(uint gpio, uint32_t events) {
//     if (events & GPIO_IRQ_EDGE_RISE) {
//         BaseType_t xHigherPriorityTaskWoken = pdFALSE;

//         // Envia o GPIO do botão que gerou a interrupção
//         xQueueSendFromISR(xBtnQueue, &gpio, &xHigherPriorityTaskWoken);
//     }
// }

// void oled_display_init(void) {
//     i2c_init(i2c1, 400000);
//     gpio_set_function(2, GPIO_FUNC_I2C);
//     gpio_set_function(3, GPIO_FUNC_I2C);
//     gpio_pull_up(2);
//     gpio_pull_up(3);

//     disp.external_vcc = false;
//     ssd1306_init(&disp, 128, 64, 0x3C, i2c1);
//     ssd1306_clear(&disp);
//     ssd1306_show(&disp);

//     gpio_init(SSD1306_PIN_LITE);
//     gpio_set_dir(SSD1306_PIN_LITE, GPIO_OUT);
//     gpio_put(SSD1306_PIN_LITE, 0);
// }

// void btns_init(void) {
//     gpio_init(BTN_PIN_R);
//     gpio_set_dir(BTN_PIN_R, GPIO_IN);
//     gpio_pull_up(BTN_PIN_R);
//     gpio_set_irq_enabled_with_callback(BTN_PIN_R,
//                                        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
//                                        true, &btn_callback);

//     gpio_init(BTN_PIN_G);
//     gpio_set_dir(BTN_PIN_G, GPIO_IN);
//     gpio_pull_up(BTN_PIN_G);
//     gpio_set_irq_enabled(BTN_PIN_G, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
//                          true);

//     gpio_init(BTN_PIN_B);
//     gpio_set_dir(BTN_PIN_B, GPIO_IN);
//     gpio_pull_up(BTN_PIN_B);
//     gpio_set_irq_enabled(BTN_PIN_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
//                          true);
// }

// void led_rgb_init(void) {
//     gpio_init(LED_PIN_R);
//     gpio_set_dir(LED_PIN_R, GPIO_OUT);
//     gpio_put(LED_PIN_R, 1);

//     gpio_init(LED_PIN_G);
//     gpio_set_dir(LED_PIN_G, GPIO_OUT);
//     gpio_put(LED_PIN_G, 1);

//     gpio_init(LED_PIN_B);
//     gpio_set_dir(LED_PIN_B, GPIO_OUT);
//     gpio_put(LED_PIN_B, 1);
// }

// static void handle_button(uint gpio) {
//     if (gpio == BTN_PIN_R) {
//         gpio_put(LED_PIN_R, 0);

//         ssd1306_clear(&disp);
//         ssd1306_draw_string(&disp, 8, 24, 2, "RED");
//         ssd1306_show(&disp);

//         vTaskDelay(pdMS_TO_TICKS(100));
//         gpio_put(LED_PIN_R, 1);
//     }
//     else if (gpio == BTN_PIN_G) {
//         gpio_put(LED_PIN_G, 0);

//         ssd1306_clear(&disp);
//         ssd1306_draw_string(&disp, 8, 24, 2, "GREEN");
//         ssd1306_show(&disp);

//         vTaskDelay(pdMS_TO_TICKS(100));
//         gpio_put(LED_PIN_G, 1);
//     }
//     else if (gpio == BTN_PIN_B) {
//         gpio_put(LED_PIN_B, 0);

//         ssd1306_clear(&disp);
//         ssd1306_draw_string(&disp, 8, 24, 2, "BLUE");
//         ssd1306_show(&disp);

//         vTaskDelay(pdMS_TO_TICKS(100));
//         gpio_put(LED_PIN_B, 1);
//     }
// }

// void task_1(void *p) {
//     oled_display_init();
//     btns_init();
//     led_rgb_init();

//     uint gpio_event;

//     while (1) {
//         if (xQueueReceive(xBtnQueue, &gpio_event, pdMS_TO_TICKS(50)) == pdTRUE) {
//             handle_button(gpio_event);
//         }
//     }
// }

// int main() {
//     stdio_init_all();

//     xBtnQueue = xQueueCreate(BTN_QUEUE_LEN, sizeof(uint));
//     if (xBtnQueue == NULL) {
//         while (1) {
//             // falha ao criar fila
//         }
//     }

//     xTaskCreate(task_1, "Task 1", 4095, NULL, 1, NULL);

//     vTaskStartScheduler();

//     while (true);
// }
