#ifndef __BC_INERTIAL_SENSOR_H__
#define __BC_INERTIAL_SENSOR_H__

#include <BC_Common.h>
#include <Arduino.h>

#define MPU6050_ADDRESS     0x68 // address pin AD0 low (GND)

//MPU6050 Gyro LPF setting
#if defined(MPU6050_LPF_256HZ) || defined(MPU6050_LPF_188HZ) || defined(MPU6050_LPF_98HZ) || defined(MPU6050_LPF_42HZ) || defined(MPU6050_LPF_20HZ) || defined(MPU6050_LPF_10HZ) || defined(MPU6050_LPF_5HZ)
  #if defined(MPU6050_LPF_256HZ)
    #define MPU6050_DLPF_CFG   0
  #endif
  #if defined(MPU6050_LPF_188HZ)
    #define MPU6050_DLPF_CFG   1
  #endif
  #if defined(MPU6050_LPF_98HZ)
    #define MPU6050_DLPF_CFG   2
  #endif
  #if defined(MPU6050_LPF_42HZ)
    #define MPU6050_DLPF_CFG   3
  #endif
  #if defined(MPU6050_LPF_20HZ)
    #define MPU6050_DLPF_CFG   4
  #endif
  #if defined(MPU6050_LPF_10HZ)
    #define MPU6050_DLPF_CFG   5
  #endif
  #if defined(MPU6050_LPF_5HZ)
    #define MPU6050_DLPF_CFG   6
  #endif
#else
    //Default settings LPF 256Hz/8000Hz sample
    #define MPU6050_DLPF_CFG   0
#endif

#define ACC_1G 512
#define ACCZ_25deg   (int16_t)(ACC_1G * 0.90631) // 0.90631 = cos(25deg) (cos(theta) of accZ comparison)
#define ACC_VelScale (9.80665f / 10000.0f / ACC_1G)
#define GYRO_SCALE (4 / 16.4 * PI / 180.0 / 1000000.0)   //MPU6050 and MPU3050   16.4 LSB/(deg/s) and we ignore the last 2 bits

#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;} 
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;} 



class BC_InertialSensor
{
public:
	BC_InertialSensor() {}
	
	int16_t		gyroADC[3];
	int16_t 	accADC[3];
	
	void gyro_init();
	void acc_init ();
	void gyro_getADC();
	void acc_getADC();
	
private:
	// gyro+acc関連変数
	int16_t		gyroZero[3];
	int16_t		accZero[3];
	uint16_t calibratingG;	// gyro
	uint16_t calibratingA;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.

	
	void gyro_common();
	void acc_common();
	void device_mag_getADC();
};

#endif // __BC_INERTIAL_SENSOR_H__
