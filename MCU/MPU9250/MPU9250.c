#include <stdint.h>
#include "./MPU9250_functions.h"
#include "./MPU9250_reg.h"
#include "./MPU9250.h"
#include "../lib/i2c.h" 
#include "../lib/USART.h"
#include "../lib/timer.h" 
#include "../Madgwick/MadgwickAHRS.h"
#include "../Madgwick/MahonyAHRS.h"
#include <math.h> 

#define GYRO_SENS 131UL
#define ACCEL_SENS 16384UL 
#define ACCEL_RES 0.000061035f //G per LSB - 4g/2^16
#define GYRO_RES 0.01526f //deg/s per LSB - 1000dps/2**16
#define MAG_RES 0.001499f //G per LSB - (2*4912uT)/2**16 = 1.499 mGefine SUCCESS          0 
#define PI 3.14159265358f 

typedef struct {
    float gyro[3];
    float accel[3];
    int32_t mag[3];
} MPU9250_calibration_values_t; 

/* These were obtained during a manual calibration routine */ 
int16_t mag_offsets[3] = {179, 20, -179};
int16_t accel_offsets[3] = {-592, 44, 17061 - 16384}; //Keep 1g in z axis
int16_t gyro_offsets[3] = {-76, 8, 60}; 

volatile MPU9250_axes_t MPU9250_axes = {0}; 
volatile MPU9250_calibration_values_t MPU9250_cal_vals = {0}; 

volatile float angles[3]; //Yaw pitch roll
/* Quarternions */
volatile float q[] = {1.0f, 0.0f, 0.0f, 0.0f};

extern uint8_t buffer[]; 

void Init_MPU9250(void) {
    /* Reset all registers */
    MPU9250_Reset(); 
    /* Enable bypass-mode to access magnetometer  */
    MPU9250_Write_Byte(MPU6050_ADDR, MPU6050_RA_INT_PIN_CFG, 0x02U);
    MPU9250_Write_Byte(MPU6050_ADDR, MPU6050_RA_INT_PIN_CFG, 0x02U);
    /* Enable the interrupt pin when raw sensor data is ready */
    MPU9250_Write_Byte(MPU6050_ADDR, MPU6050_RA_INT_ENABLE, 0x01U);
    /* Setup the gyroscope digital low0pass filter (DLPF)  with FIFO setting */
    MPU9250_Write_Byte(MPU6050_ADDR, MPU6050_RA_CONFIG, 0x03U);
    /* Set gyroscope sensitivity to +- 500 dps */ 
    MPU9250_Write_Byte(MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG, 0x08U);
    /* Set accelerometer sensitivity to +- 2 g */ 
    MPU9250_Write_Byte(MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG_1, 0x00U);
    /* Set the accelerometer DLPF settings */ 
    MPU9250_Write_Byte(MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG_2, 0x03U);
    /* Set the sample-rate division */
    MPU9250_Write_Byte(MPU6050_ADDR, MPU6050_RA_SMPLRT_DIV, 0x04U); 
}

void Init_AK8963(void) {
    /* Activate power-down mode and initialize read-only registers */
    TIM2_Delay_ms(1);
    MPU9250_Write_Byte(AK8963_ADDR, AK8963_CNTL_1, 0x00U);
    /* Allow fuse-rom access */
    TIM2_Delay_ms(1);
    MPU9250_Write_Byte(AK8963_ADDR, AK8963_CNTL_1, 0x0FU);
    /* Read calibration data */ 
    TIM2_Delay_ms(1);
    MPU9250_Read_Bytes(AK8963_ADDR, AK8963_ASAX, 3, buffer);
    MPU9250_cal_vals.mag[0] = (buffer[0]-128)/256 + 1;
    MPU9250_cal_vals.mag[1] = (buffer[1]-128)/256 + 1;
    MPU9250_cal_vals.mag[2] = (buffer[2]-128)/256 + 1;

    /* Remove fuse-rom access again */ 
    /*X Configure AK8963 to return 16 bit values with 100 Hz sample rate */
    MPU9250_Write_Byte(AK8963_ADDR, AK8963_CNTL_1, 0x00U);
    TIM2_Delay_ms(1);
    MPU9250_Write_Byte(AK8963_ADDR, AK8963_CNTL_1, 0x11U);
    TIM2_Delay_ms(1);
}

void MPU6050_Poll_Axis() { 
    /* Burst read raw sensor values*/
    MPU9250_Read_Bytes(MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
    /* Wait until done as the read is asynchronous */
    I2C_Wait_Until_Done(); 
    /* Load the values into the global axes struct */
    MPU9250_axes.x.accel = ((buffer[0] << 8) | buffer[1]) - accel_offsets[0];
    MPU9250_axes.y.accel = ((buffer[2] << 8) | buffer[3]) - accel_offsets[1];
    MPU9250_axes.z.accel = ((buffer[4] << 8) | buffer[5]) - accel_offsets[2];
    /* Throw away temperature measurement */
    MPU9250_axes.x.gyro = ((buffer[8 ] << 8) | buffer[9 ]) - gyro_offsets[0];
    MPU9250_axes.y.gyro = ((buffer[10] << 8) | buffer[11]) - gyro_offsets[1];
    MPU9250_axes.z.gyro = ((buffer[12] << 8) | buffer[13]) - gyro_offsets[2];
}

void AK8963_Poll_Axis() { 
    if (MPU9250_Read_Byte(AK8963_ADDR, AK8963_ST1)) {
        /* Burst read raw sensor values */
        MPU9250_Read_Bytes(AK8963_ADDR, AK8963_XOUT_L, 7, buffer);
        if (buffer[7] == 0x08) { 
            //Can't use measurements!  
            MPU9250_axes.x.mag = 0U;
            MPU9250_axes.y.mag = 0U;
            MPU9250_axes.z.mag = 0U; 
            MPU9250_Write_Byte(AK8963_ADDR, AK8963_CNTL_1, 0x11U); 
        } else { 
            /* Wait until done as the read is asynchronous */
            I2C_Wait_Until_Done();
            /* Load the values into the global axes struct */
            /* The order is switched to align with MPU6050 */
            MPU9250_axes.y.mag = (buffer[0] | (buffer[1] << 8)) *
                MPU9250_cal_vals.mag[1] - mag_offsets[1];
            MPU9250_axes.x.mag = (buffer[2] | (buffer[3] << 8)) *
                MPU9250_cal_vals.mag[0] - mag_offsets[0]; 
            MPU9250_axes.z.mag = (buffer[4] | (buffer[5] << 8)) *
                MPU9250_cal_vals.mag[2] - mag_offsets[2];

            /* If single measurement mode then start another one */
            MPU9250_Write_Byte(AK8963_ADDR, AK8963_CNTL_1, 0x11U); 

        }
    } else { 
        /* This indicates that the 6-DOF sensor fusion should be used */
        MPU9250_axes.x.mag = 0U;
        MPU9250_axes.y.mag = 0U;
        MPU9250_axes.z.mag = 0U; 
        MPU9250_Write_Byte(AK8963_ADDR, AK8963_CNTL_1, 0x11U); 
    } 
}

void MPU9250_Fusion(float sample_frequency) {
    /* Form scientific SI values */ 
    float ax = ((float)MPU9250_axes.x.accel)*ACCEL_RES;
    float ay = ((float)MPU9250_axes.y.accel)*ACCEL_RES;
    float az = ((float)MPU9250_axes.z.accel + 16384)*ACCEL_RES;
    float gx = ((float)MPU9250_axes.x.gyro)*(GYRO_RES*PI/180.f);
    float gy = ((float)MPU9250_axes.y.gyro)*(GYRO_RES*PI/180.f);
    float gz = ((float)MPU9250_axes.z.gyro)*(GYRO_RES*PI/180.f); 
    float mx = ((float)MPU9250_axes.x.mag)*MAG_RES;
    float my = ((float)MPU9250_axes.y.mag)*MAG_RES;
    float mz = ((float)MPU9250_axes.z.mag)*MAG_RES; 

    /* Sensor fusion */
    MadgwickAHRSupdate(gx, gy, gz, ax,  ay,  az,  mx,  my,  mz, sample_frequency); 
    //MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az, sample_frequency);
    //MahonyAHRSupdate(gx, gy, gz, ax,  ay,  az,  mx,  my,  mz, sample_frequency); 
    //MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az, sample_frequency); 

    /* Quaternion to Tait-Bryan angles */
    angles[0] = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]); 
    angles[1] = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    angles[2] = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]); 
    /* Change to 16-bit resolution */
    angles[0] *= 32768.f/(2.f*PI);
    angles[1] *= 32768.f/(2.f*PI);
    angles[2] *= 32768.f/(2.f*PI); 
}

void MPU6050_Calibration_Setup(void) {
    /* Reset all registers */
    MPU9250_Reset();
    /* Disable Interrupts */
    MPU9250_Write_Byte(MPU6050_ADDR, MPU6050_RA_INT_ENABLE, 0x00U);    
    /* Disable FIFO */
    MPU9250_Write_Byte(MPU6050_ADDR, MPU6050_RA_FIFO_EN, 0x00U);    
    /* Internal Clock Source */
    MPU9250_Write_Byte(MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, 0x00U);    
    /* Disable I2C master */
    MPU9250_Write_Byte(MPU6050_ADDR, MPU6050_RA_I2C_MST_CTRL, 0x00U);    
    /*Disable FIFO and I2C master modes */
    MPU9250_Write_Byte(MPU6050_ADDR, MPU6050_RA_USER_CTRL, 0x00U);
    /* Reset FIFO */
    MPU9250_Write_Byte(MPU6050_ADDR, MPU6050_RA_USER_CTRL, 0x04);    
    TIM2_Delay_ms(15);

    /* Configure sensors for bias calculation */
    /* 184 Hz DLPF and FIFO until full */
    MPU9250_Write_Byte(MPU6050_ADDR, MPU6050_RA_CONFIG, 0x41);
    /* 1 kHz sample-rate */
    MPU9250_Write_Byte(MPU6050_ADDR, MPU6050_RA_SMPLRT_DIV, 0x00U);    
    /* Set gyroscope sensitivity to +- 25 dps */ 
    MPU9250_Write_Byte(MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG, 0x00U);
    /* Set accelerometer sensitivity to +- 2 g */ 
    MPU9250_Write_Byte(MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG_1, 0x00U); 
} 

void MPU6050_Calibration(void) { 
    MPU6050_Calibration_Setup(); 

    int32_t gyro_arr[] = {0, 0, 0};  
    int32_t gyro_mean[3];
    int32_t accel_arr[] = {0, 0, 0};  
    int32_t accel_mean[3];
    uint16_t fifo_cnt; 

    /* Enable accelerometer and gyro FIFO */
    MPU9250_Write_Byte(MPU6050_ADDR, MPU6050_RA_FIFO_EN, 0x78U);
    /* Enable FIFO */
    MPU9250_Write_Byte(MPU6050_ADDR, MPU6050_RA_USER_CTRL, 0x40U);
    TIM2_Delay_ms(42); //504 bytes of values should have accumulated
    /* Disable FIFO */
    MPU9250_Write_Byte(MPU6050_ADDR, MPU6050_RA_USER_CTRL, 0x00U);
    /* Read number of values in FIFO */ 
    TIM2_Delay_ms(1); //Seems to be necessary
    fifo_cnt = MPU9250_Read_Word(MPU6050_ADDR, MPU6050_RA_FIFO_COUNTH); 
    /* Split into sets of 12 */
    fifo_cnt /= 12U; 

    /* We now burst read from the FIFO 12 bytes at a time and form cumulative
     * means */
    for (uint8_t i = 0; i < fifo_cnt; i++) {
        MPU9250_Read_Bytes(MPU6050_ADDR, MPU6050_RA_FIFO_R_W, 12, buffer);
        I2C_Wait_Until_Done();
        accel_arr[0] += (int32_t)((buffer[i+0 ] << 8U)) | buffer[i+1];
        accel_arr[1] += (int32_t)((buffer[i+2 ] << 8U)) | buffer[i+3];
        accel_arr[2] += (int32_t)((buffer[i+4 ] << 8U)) | buffer[i+5];
        gyro_arr[0]  += (int32_t)((buffer[i+6 ] << 8U)) | buffer[i+7];
        gyro_arr[1]  += (int32_t)((buffer[i+8 ] << 8U)) | buffer[i+9];
        gyro_arr[2]  += (int32_t)((buffer[i+10] << 8U)) | buffer[i+11]; 
    }

    /* Now calculate the means and convert to actual units */
    for (uint8_t i = 0; i < 3; i++) {
        accel_mean[i] = accel_arr[i]/fifo_cnt; 
        gyro_mean[i] = gyro_arr[i]/fifo_cnt; 
        /* Convert to actual units reusing the last array variables */
        accel_arr[i] = accel_mean[i]/ACCEL_SENS;
        gyro_arr[i] = gyro_mean[i]/GYRO_SENS;
    }

    for (uint8_t i = 0; i < 3; i++) { 
        MPU9250_cal_vals.gyro[i] = (float)gyro_mean[i]/(float)GYRO_SENS; 
        MPU9250_cal_vals.accel[i] = (float)accel_mean[i]/(float)ACCEL_SENS; 
    } 
} 

void MPU9250_Zero_Mag(void) {
        MPU9250_axes.x.mag = 0U;
        MPU9250_axes.y.mag = 0U;
        MPU9250_axes.z.mag = 0U; 
}
