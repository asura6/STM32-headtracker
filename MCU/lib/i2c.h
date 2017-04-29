#ifndef STM_I2C_
#define STM_I2C_

typedef struct {
    uint8_t byte0;
    uint8_t byte1;
} I2C_word_t; 

void Init_I2C(void);
void I2C_Restart(void);
void I2C_Request_Stop(void);


void I2C_Send_Byte(uint8_t address, uint8_t byte);
void I2C_Read_Byte(uint8_t address);
//uint16_t I2C_Read_Word(uint8_t address);
void I2C_Send_Bytes(uint8_t address, uint32_t nr_bytes, uint8_t *DMA_buffer);
void I2C_Read_Bytes(uint8_t address, uint32_t nr_bytes, uint8_t *DMA_buffer);
void I2C_Init_DMA(void );
void I2C_Wait_Until_Ready(void );
void I2C_Wait_Until_Done(void );
uint8_t I2C_Get_Byte(void);


#endif
