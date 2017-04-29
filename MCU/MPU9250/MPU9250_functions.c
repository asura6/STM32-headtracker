#include <stdint.h>
#include "MPU9250_functions.h"
#include "MPU9250_reg.h"
#include "../lib/timer.h"
#include "../lib/i2c.h"
#include "../lib/USART.h" 

uint8_t buffer[14]; //Maximum reasonable burst-read size 

void MPU9250_Write_Word(uint8_t addr, uint8_t regAddr, uint16_t word) { 
    buffer[0] = regAddr;
    buffer[1] = (uint8_t)(word & 0x00FFU);
    buffer[2] = (uint8_t)((word & 0xFF00U) >> 8U); 
    I2C_Send_Bytes(addr, 4, buffer); 
}

void MPU9250_Write_Byte(uint8_t addr, uint8_t regAddr, uint8_t byte) {
    buffer[0] = regAddr;
    buffer[1] = byte;
    I2C_Send_Bytes(addr, 3, buffer); 
}

void MPU9250_Write_Bytes( uint8_t addr, uint8_t startAddr, uint8_t nrBytes,
        uint8_t *input_bytes) {
    /* Copy to the static array */
    buffer[0] = startAddr;
    for (uint8_t i = 1; i <= nrBytes; i++) {
        buffer[i] = input_bytes[i - 1];
    } 
    I2C_Send_Bytes(addr, nrBytes + 2, buffer); 
} 

uint16_t MPU9250_Read_Word(uint8_t addr, uint8_t regAddr) {
    MPU9250_Read_Bytes(addr, regAddr, 2, buffer);
    I2C_Wait_Until_Done();
    return (buffer[1] | (buffer[0] << 8));
}


uint8_t MPU9250_Read_Byte(uint8_t addr, uint8_t regAddr) {
    I2C_Send_Byte(addr, regAddr); 
    I2C_Read_Byte(addr); 
    I2C_Wait_Until_Done();
    return I2C_Get_Byte(); 
}

void MPU9250_Read_Bytes(uint8_t addr, uint8_t startAddr, uint8_t nrBytes, 
        uint8_t *output_bytes) {
    I2C_Send_Byte(addr, startAddr);
    I2C_Read_Bytes(addr, nrBytes, output_bytes); 
}

/* Resets all registers */
void MPU9250_Reset(void) { 
    MPU9250_Write_Byte(MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, 0x81); 
    /* This sequence makes the protocol analyzer happy */
    TIM2_Delay_ms(20);
    I2C_Request_Stop();
    TIM2_Delay_ms(20);
    I2C_Restart();
    TIM2_Delay_ms(60); 
} 


