#ifndef MPU9250_CORE
#define MPU9250_CORE

typedef struct {
    struct {
        int16_t accel;
        int16_t gyro;
        int16_t mag;
    } x;
    struct {
        int16_t accel;
        int16_t gyro;
        int16_t mag;
    } y;
    struct {
        int16_t accel;
        int16_t gyro;
        int16_t mag;
    } z; 
} MPU9250_axes_t; 

void Init_MPU9250(void);
void Init_AK8963(void);
void MPU6050_Poll_Axis();
void AK8963_Poll_Axis();
void MPU6050_Calibration(void); 
void MPU9250_Fusion(void);


#endif
