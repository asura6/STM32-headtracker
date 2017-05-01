#include <stm32f10x.h>
#include <stdint.h>
#include "./timer.h"

volatile uint32_t TIM2_counter = 0;
volatile uint32_t TIM3_NOT_FINISHED = 1; 
volatile uint32_t TIM4_counter = 0;

void Init_TIM2(void) {
    /* Enable clock to the TIM2 module */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; 
    /* Set the prescaler and auto-reload value to cause the timer to trigger
     * once every ms. The larger auto-reload value we have the better resolution
     * the timer has*/
    TIM2->PSC = 1U;
    TIM2->ARR = 36000U;
    /* Enable update interrupt */
    TIM2->DIER = TIM_DIER_UIE;
    /* Enable NVIC */
    NVIC_EnableIRQ(TIM2_IRQn); 
} 

void TIM2_Delay_ms(uint32_t ms) {
    TIM2_counter = 0;
    /* Reset counter */
    TIM2->CNT = 0x00U;
    /* Enable TIM2 counter */
    TIM2->CR1 |= TIM_CR1_CEN;

    while (TIM2_counter < ms) {
        /* Wait until count is complete */
        __asm("WFI");
    }
    /* Disable TIM2 counter */
    TIM2->CR1 &= ~TIM_CR1_CEN;
} 

void Init_TIM3(uint16_t ms) {
    /* NOTE! Maximum delay is 10 ms */

    /* Enable clock to the TIM3 module */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; 
    /* Enable update interrupt */
    TIM3->DIER = TIM_DIER_UIE;
    /* Reset and set up counter */
    TIM3->PSC = 11U;
    TIM3->ARR = 6000U*ms;
    TIM3->CNT = 0x00U; 
    /* Enable NVIC */
    NVIC_EnableIRQ(TIM3_IRQn); 
} 

void TIM3_Start(void) { 
    /* Enable TIM3 counter */
    TIM3->CR1 |= TIM_CR1_CEN; 
}

void TIM3_Reset(void) {
    /* Reset counter */
    TIM3->CNT = 0x00U; 
    TIM3_NOT_FINISHED = 1;
}

void TIM3_Wait(void) {
    while (TIM3_NOT_FINISHED) {
        /* Wait until time has passed */
        __asm("WFI");
    }
    /* Reset counter */
    TIM3->CNT = 0x00U;
    TIM3_NOT_FINISHED = 1;
} 

uint8_t TIM3_Is_Counting() {
    return TIM3_NOT_FINISHED;
}

void Init_TIM4(void) {
    /* Enable clock to the TIM3 module */
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; 
    /* Enable update interrupt */
    TIM4->DIER = TIM_DIER_UIE;
    /* Reset and set up counter */
    TIM4->PSC = 0U;
    TIM4->ARR = 720U; //1 us resolution
    TIM4->CNT = 0x00U; 
    /* Enable NVIC */
    NVIC_EnableIRQ(TIM4_IRQn); 
}

void TIM4_Start(void) {
    /* Enable TIM4 counter */
    TIM4->CR1 |= TIM_CR1_CEN; 
}

void TIM4_Reset(void) {
    /* Reset counter */
    TIM4->CNT = 0x00U;
    TIM4_counter = 0;
}

/* Get elapsed time in microseconds */
uint32_t TIM4_Get_Elapsed(void) {
    if (TIM4_counter == 0) {
        return 10;
    }
    return TIM4_counter;
} 

void TIM2_IRQ(void) {
    /* Reset the interrupt flag and increment the global counter */
    TIM2->SR &= ~TIM_SR_UIF;
    TIM2_counter++;
}

void TIM3_IRQ(void) {
    /* Reset the interrupt flag */
    TIM3->SR &= ~TIM_SR_UIF;
    TIM3_NOT_FINISHED = 0;
}

void TIM4_IRQ(void) {
    /* Reset the interrupt flag and increment the global counter */
    TIM4->SR &= ~TIM_SR_UIF;
    TIM4_counter++;
}
