/* USB HID Mouse for STM32F103C8
 * Robin Isaksson 2017 */
#include <stm32f10x.h>
#include <stdint.h>
#include "lib/system_clock.h"
#include "lib/timer.h"
#include "lib/gpio.h"
#include "lib/USART.h"
#include "lib/i2c.h"
#include "lib/USB.h"
#include "lib/USB_conf.h"
#include "MPU9250/MPU9250_reg.h"
#include "MPU9250/MPU9250.h" 
#include "Madgwick/MadgwickAHRS.h" 

/*********************
 ***** CONSTANTS *****
 *********************/

#define CPU_CLK 72000000U
#define USART_BAUD_RATE 115200U

/************************
***** BEGIN PROGRAM *****
*************************/

extern volatile HID_report_t HID_report;
extern volatile MPU9250_axes_t MPU9250_axes; 
extern volatile float angles[3]; 

int main(void) { 
    NVIC_SetPriority(TIM2_IRQn, 10);
    NVIC_SetPriority(I2C1_EV_IRQn, 9);
    NVIC_SetPriority(USART1_IRQn, 8);
    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0); 

    Init_System_Clocks();                       //Initialize system clocks
    Init_GPIO();                                //Initialize IO pins
    Init_TIM2();                                //Initialize Timer 2
    NVIC_EnableIRQ(TIM2_IRQn);                  //Enable timer 2 interrupts
    Init_USART(CPU_CLK, USART_BAUD_RATE);       //Initialize USART
    USART_Clear_Screen();                       //Clear the hyperterminal
    USART_Send_Str("\n\rUSART initialized...");
    Init_I2C();                                 //Initialize I2C
    USART_Send_Str("\n\rI2C initialized..."); 
    I2C_Init_DMA();                             //Initialize the I2C-DMA 
    USART_Send_Str("\n\rDMA initialized..."); 
    Init_USB();                                 //Initialize USB
    USART_Send_Str("\n\rUSB initialized..."); 

    while(USB_Is_Not_Ready()) {
        /* Wait until the device is ready to transmit HID reports */
    }; 

    //MPU6050_Calibration();
    Init_MPU9250(); 
    Init_AK8963(); 
    TIM2_Delay_ms(100); 
    while(1) { 
        MPU6050_Poll_Axis(); 
        AK8963_Poll_Axis(); 
        MPU9250_Fusion(); 
        /* Update values to be sent in the USB-packet */
        //HID_report.t_x = (int16_t)(angles[0]); 
        //HID_report.t_y = (int16_t)(angles[1]); 
        //HID_report.t_z = (int16_t)(angles[2]); 
        HID_report.t_x = MPU9250_axes.x.gyro; 
        HID_report.t_y = MPU9250_axes.y.gyro; 
        HID_report.t_z = MPU9250_axes.z.gyro; 
        HID_report.r_x = MPU9250_axes.x.accel; 
        HID_report.r_y = MPU9250_axes.y.accel; 
        HID_report.r_z = MPU9250_axes.z.accel; 
        HID_report.m_x = MPU9250_axes.x.mag; 
        HID_report.m_y = MPU9250_axes.y.mag; 
        HID_report.m_z = MPU9250_axes.z.mag; 
        HID_report.buttons = 0x00U;

        /* Update the HID report and transmit */
        USB_Update_HID_Report(); 
        /* Delay 50 ms and toggle the on-board LED */
        TIM2_Delay_ms(10U);
        LED_Toggle(GPIOC_BASE, 13);
    }
}
