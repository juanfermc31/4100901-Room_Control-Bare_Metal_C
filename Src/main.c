/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Sam C
 * @brief          : Main program body
 ******************************************************************************
 */
#include "gpio.h"
#include "systick.h"
#include "nvic.h"
#include "uart.h"
#include "tim.h"
#include "room_control.h"

// Variable global para control de apagado del LED externo tras 3s
extern volatile uint32_t led_on_time_ms;

void heartbeat_led_toggle(void)
{
    static uint32_t last_tick = 0;
    if (systick_get_tick() - last_tick >= 500) { // Cambia cada 500 ms
        gpio_toggle_pin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
        last_tick = systick_get_tick();
    }
}

/**
 * @brief Función principal del programa.
 *        Configura los periféricos y entra en un bucle infinito.
 *        El LED de heartbeat parpadea cada 500 ms.
 *        El LED externo se enciende 3 segundos tras pulsar el botón.
 */
int main(void)
{
    // Inicialización de SysTick
    systick_init_1ms(); // Usa SYSCLK_FREQ_HZ (ej. 4MHz) definido en rcc.h

    // Configuración del LED Heartbeat (PA5)
    gpio_setup_pin(GPIOA, HEARTBEAT_LED_PIN, GPIO_MODE_OUTPUT, 0);

    // Configuración LED externo ON/OFF (PA7)
    gpio_setup_pin(GPIOA, EXTERNAL_LED_ONOFF_PIN, GPIO_MODE_OUTPUT, 0);
    gpio_write_pin(EXTERNAL_LED_ONOFF_PORT, EXTERNAL_LED_ONOFF_PIN, GPIO_PIN_RESET); // Asegura apagado inicial

    // Configuración del botón B1 (PC13)
    gpio_setup_pin(GPIOC, USER_BUTTON_PIN, GPIO_MODE_INPUT, 0);
    nvic_exti_pc13_button_enable();  // Habilita interrupción EXTI para PC13

    // Inicialización USART2 para comunicación serial
    uart2_init(115200);
    nvic_usart2_irq_enable();

    // Inicialización TIM3 Canal 1 para PWM (por ejemplo para otro LED)
    tim3_ch1_pwm_init(1000);      // 1000 Hz
    tim3_ch1_pwm_set_duty_cycle(70);  // 70%

    // Inicialización de la lógica de control de la aplicación
    room_control_app_init();

    // Mensaje inicial por UART
    uart2_send_string("\r\nSistema Inicializado. Esperando eventos...\r\n");

    // Bucle infinito
    while (1) {
        // Parpadeo LED Heartbeat cada 500 ms
        heartbeat_led_toggle();

        // Control de apagado automático del LED externo a los 3 segundos
        if (led_on_time_ms != 0 && systick_get_tick() >= led_on_time_ms) {
            gpio_write_pin(EXTERNAL_LED_ONOFF_PORT, EXTERNAL_LED_ONOFF_PIN, GPIO_PIN_RESET);
            led_on_time_ms = 0;
        }

        // Aquí puedes agregar otras tareas o lógicas no bloqueantes
        // Ejemplo: room_control_app_loop();
    }
}
