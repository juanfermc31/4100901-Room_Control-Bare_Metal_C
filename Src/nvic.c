/**
 ******************************************************************************
 * @file           : nvic.c
 * @author         : Sam C
 * @brief          : NVIC driver for STM32L476RGTx
 ******************************************************************************
 */
#include "nvic.h"
#include "rcc.h"  // Para habilitar relojes de GPIOC y SYSCFG
#include "uart.h" // Para acceso a USART2

#define NVIC_ENABLE_IRQ(IRQn) (NVIC->ISER[(IRQn) / 32U] |= (1UL << ((IRQn) % 32U)))

void nvic_exti_pc13_button_enable(void)
{
    // 1. Habilitar el reloj para SYSCFG
    rcc_syscfg_clock_enable(); // SYSCFG necesario para mapear EXTI a GPIO

    // 2. Configurar línea EXTI13 (SYSCFG_EXTICR)
    SYSCFG->EXTICR[3] &= ~(0x000FU << 4);  // Limpiar campo EXTI13 (bits 7-4)
    SYSCFG->EXTICR[3] |=  (0x0002U << 4);  // Conectar EXTI13 a PC13 (0b0010 para PC)

    // 3. Habilitar interrupción para línea EXTI13 (IMR1)
    EXTI->IMR1 |= (1U << 13);

    // 4. Configurar trigger flanco de bajada (FTSR1) y deshabilitar flanco de subida (RTSR1)
    EXTI->FTSR1 |= (1U << 13);   // Habilitar flanco de bajada
    EXTI->RTSR1 &= ~(1U << 13);  // Deshabilitar flanco de subida

    // 5. Habilitar IRQ EXTI15_10 en NVIC
    NVIC_ENABLE_IRQ(EXTI15_10_IRQn);
}

void nvic_usart2_irq_enable(void)
{
    // Habilitar interrupción RXNEIE (bit 5 en CR1 del USART2)
    USART2->CR1 |= (1U << 5);

    // Habilitar IRQ USART2 en NVIC
    NVIC_ENABLE_IRQ(USART2_IRQn);
}
