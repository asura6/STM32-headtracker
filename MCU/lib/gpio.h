#ifndef STM_GPIO
#define STM_GPIO

void Init_GPIO(void);
void LED_Toggle(uint32_t port, uint32_t pin);
void Init_PB0_Interrupt(void);


#endif
