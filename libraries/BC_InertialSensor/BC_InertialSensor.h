/** charset=UTF-8 **/

#ifndef __BC_INERTIAL_SENSOR_H__
#define __BC_INERTIAL_SENSOR_H__

#include <BC_Common.h>
#include <BC_I2C.h>
#include <Arduino.h>

// ***********************************************************************************
// defines
// ***********************************************************************************
#define ACC_1G 512
#define ACCZ_25deg   (int16_t)(ACC_1G * 0.90631) // 0.90631 = cos(25deg) (cos(theta) of accZ comparison)
#define ACC_VelScale (9.80665f / 10000.0f / ACC_1G)
#define GYRO_SCALE (4 / 16.4 * PI / 180.0 / 1000000.0)   //MPU6050 and MPU3050   16.4 LSB/(deg/s) and we ignore the last 2 bits

// BC_Compassから読み込めるようにヘッダファイルに書く
#define MPU6050_ADDRESS     0x68 // address pin AD0 low (GND)
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;} 
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;} 


// ***********************************************************************************
// class
// ***********************************************************************************
class BC_InertialSensor
{
public:
	BC_InertialSensor(BC_I2C &i2c):
		calibratingG(0),
		calibratingA(0),
		calib_OK_G(false),
		calib_OK_A(false),
		_i2c(i2c)
	{}
	
	uint8_t		rawADC[6];
	int16_t		gyroADC[3];
	int16_t 	accADC[3];
	
	void init();
	void gyro_init();
	void acc_init ();
	
	void get_data();
	void gyro_getADC();
	void acc_getADC();
	
	void calib_start();
	void gyro_calib_start();
	void acc_calib_start();
	
	bool calib_ok(){return (gyro_calib_ok() && acc_calib_ok());}
	bool gyro_calib_ok(){return calib_OK_G;}
	bool acc_calib_ok(){return calib_OK_A;}
	
	
private:
	int16_t		gyroZero[3];
	int16_t		accZero[3];
	uint16_t	calibratingG;	// gyro
	uint16_t	calibratingA;	// the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
	bool		calib_OK_G;
	bool		calib_OK_A;
	
	void gyro_common();
	void acc_common();
	//void device_mag_getADC();
	
protected:
	BC_I2C	&_i2c;
};

#endif // __BC_INERTIAL_SENSOR_H__
