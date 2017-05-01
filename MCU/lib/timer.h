#ifndef STM_TIMER
#define STM_TIMER

void Init_TIM2(void);
void TIM2_Delay_ms(uint32_t ms);
void Init_TIM3(uint16_t ms);
void TIM3_Start(void);
void TIM3_Reset(void); 
void TIM3_Wait(void); 
uint8_t TIM3_Is_Counting(); 
void Init_TIM4(void);
void TIM4_Start(void);
void TIM4_Reset(void);
uint32_t TIM4_Get_Elapsed(void);

#endif
