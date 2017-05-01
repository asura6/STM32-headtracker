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
#define USB_UPDATE_DELAY 10U //Time elapsed in ms between prepared USB report

/************************
***** BEGIN PROGRAM *****
*************************/

extern volatile HID_report_t HID_report;
extern volatile MPU9250_axes_t MPU9250_axes; 
extern volatile float angles[3]; 
uint32_t sample_period;
uint8_t sample_counter;
uint8_t data_not_ready = 0;

int main(void) { 
    NVIC_SetPriority(TIM2_IRQn, 10);
    NVIC_SetPriority(I2C1_EV_IRQn, 9);
    NVIC_SetPriority(USART1_IRQn, 8);
    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0); 

    Init_System_Clocks();                       //Initialize system clocks
    Init_GPIO();                                //Initialize IO pins 
    Init_PB0_Interrupt(); 
    Init_TIM2();                                //Initialize Timer 2 
    Init_TIM3(USB_UPDATE_DELAY);                //Initialize Timer 3
    Init_TIM4();                                //Initialize Tiemr 4

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
    TIM3_Start(); //Keep track on when to send USB-packets
    TIM4_Start(); //Keep track on the sampling rate
    while(1) { 
        /* Loop until USB-packet ready to be sent */
        while (TIM3_Is_Counting()) {
            /* Use last timer count to calculate the sample rate */
            float sample_frequency = 1.f/((((float)TIM4_Get_Elapsed())/1000000.f));

            TIM4_Reset();

            /* Wait until data is ready */
            while (data_not_ready) {
                __ASM("WFI");
            } 

            /* Read accelerometer and gyroscope values */
            MPU6050_Poll_Axis(); 

            /* Only try sampling the magnetometer every third time  */
            if (sample_counter >= 2) { 
                AK8963_Poll_Axis(); 
                sample_counter = 0;
            } else {
                MPU9250_Zero_Mag(); //Set to zero to use 6-DOF algorithm
            }
            data_not_ready = 1; //Block sampling until sensors are ready

            /* Perform sensor fusion */
            MPU9250_Fusion(sample_frequency);
            sample_counter++; 
        }
        /* Update values to be sent in the USB-packet */
        HID_report.t_x = MPU9250_axes.x.gyro; 
        HID_report.t_y = MPU9250_axes.y.gyro; 
        HID_report.t_z = MPU9250_axes.z.gyro; 

        HID_report.r_x = MPU9250_axes.x.accel; 
        HID_report.r_y = MPU9250_axes.y.accel; 
        HID_report.r_z = MPU9250_axes.z.accel; 

        HID_report.m_x = MPU9250_axes.x.mag; 
        HID_report.m_y = MPU9250_axes.y.mag; 
        HID_report.m_z = MPU9250_axes.z.mag; 

        HID_report.yaw = (int16_t)(angles[0]); 
        HID_report.pitch = (int16_t)(angles[1]); 
        HID_report.roll = (int16_t)(angles[2]); 

        HID_report.buttons = 0x00U; 

        /* Update the HID report and transmit */ 
        USB_Update_HID_Report(); 
        LED_Toggle(GPIOC_BASE, 13);
        TIM3_Reset();
    }
} 

void EXTI0_IRQ(void) { 
    data_not_ready = 0;
    //Clear pending interrupt 
    EXTI->PR |= 0x01U;
    //USART_Send_Str("\n\rMPU9250 data ready"); 
} 
