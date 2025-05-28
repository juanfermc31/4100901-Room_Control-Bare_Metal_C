/**
 ******************************************************************************
 * @file           : gpio.c
 * @author         : Sam C
 * @brief          : GPIO driver for STM32L476RGTx
 ******************************************************************************
 */
#include "gpio.h"
#include "rcc.h"
#include "nvic.h"
#include "systick.h"  // Para systick_get_tick()

// Variable global para controlar apagado del LED externo después de 3 segundos
volatile uint32_t led_on_time_ms = 0;

void gpio_setup_pin(GPIO_TypeDef *gpio_port, uint8_t pin_number,
                    uint8_t mode, uint8_t alternate_function)
{
    // 1. Habilitar el reloj para el puerto GPIO correspondiente
    rcc_gpio_clock_enable(gpio_port);

    // 2. Configurar el modo del pin (Input, Output, AF, Analog)
    gpio_port->MODER &= ~(0x03U << (pin_number * 2));
    gpio_port->MODER |= (mode << (pin_number * 2));

    // 3. Configurar función alternativa si es AF
    if (mode == GPIO_MODE_AF) {
        uint32_t temp_af_val = alternate_function;
        if (pin_number < 8) {
            gpio_port->AFRL &= ~(0x0FU << (pin_number * 4));
            gpio_port->AFRL |= (temp_af_val << (pin_number * 4));
        } else {
            gpio_port->AFRH &= ~(0x0FU << ((pin_number - 8) * 4));
            gpio_port->AFRH |= (temp_af_val << ((pin_number - 8) * 4));
        }
    }
}

void gpio_write_pin(GPIO_TypeDef *gpio_port, uint8_t pin_number, uint8_t pin_state)
{
    if (pin_state == GPIO_PIN_SET) {
        gpio_port->BSRR = (1U << pin_number);
    } else {
        gpio_port->BSRR = (1U << (pin_number + 16));
    }
}

void gpio_toggle_pin(GPIO_TypeDef *gpio_port, uint8_t pin_number)
{
    gpio_port->ODR ^= (1U << pin_number);
}

uint8_t gpio_read_pin(GPIO_TypeDef *gpio_port, uint8_t pin_number)
{
    return ((gpio_port->IDR >> pin_number) & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

/**
 * @brief Rutina de Servicio de Interrupción para EXTI líneas 10 a 15.
 *        Este nombre debe coincidir exactamente con el definido en la tabla de vectores
 *        del archivo de arranque (startup_stm32l476rgtx.s).
 *        Esta ISR puede ser llamada por room_control.c si la lógica es compleja.
 */
void EXTI15_10_IRQHandler(void) {
    // 1. Verificar si la interrupción fue de la línea EXTI13
    if ((EXTI->PR1 & (1U << USER_BUTTON_PIN)) != 0) {
        // 2. Limpiar el flag de pendiente de la interrupción (escribiendo '1')
        EXTI->PR1 |= (1U << USER_BUTTON_PIN);

        // 3. Encender LED externo (PA7)
        gpio_write_pin(EXTERNAL_LED_ONOFF_PORT, EXTERNAL_LED_ONOFF_PIN, GPIO_PIN_SET);

        // 4. Marcar tiempo para apagar LED luego de 3 segundos
        led_on_time_ms = systick_get_tick() + 3000;
    }
}

