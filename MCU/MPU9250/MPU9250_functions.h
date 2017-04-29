#ifndef MPU9250_FUNCTIONS
#define MPU9250_FUNCTIONS

void MPU9250_Set_Bit(uint8_t addr, uint8_t regAddr, uint8_t bit);
void MPU9250_Clear_Bit(uint8_t addr, uint8_t regAddr, uint8_t bit);
void MPU9250_Write_Byte(uint8_t addr, uint8_t regAddr, uint8_t byte); 
void MPU9250_Write_Bytes(uint8_t addr, uint8_t startAddr, uint8_t nrBytes,
        uint8_t *input_bytes);
uint8_t MPU9250_Read_Byte(uint8_t addr, uint8_t regAddr); 
uint16_t MPU9250_Read_Word(uint8_t addr, uint8_t regAddr);
void MPU9250_Read_Bytes(uint8_t addr, uint8_t startAddr, uint8_t nrBytes, 
        uint8_t *output_bytes); 
void MPU9250_Write_Word(uint8_t addr, uint8_t regAddr, uint16_t word);
void MPU9250_Reset(void); 

#endif
