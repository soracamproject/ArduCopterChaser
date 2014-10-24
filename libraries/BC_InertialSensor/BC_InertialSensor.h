/** charset=UTF-8 **/

#ifndef __BC_INERTIAL_SENSOR_H__
#define __BC_INERTIAL_SENSOR_H__

#include <BC_Common.h>
#include <BC_Math.h>
#include <BC_I2C.h>

// ***********************************************************************************
// defines
// ***********************************************************************************
// BC_Compassから読み込めるようにヘッダファイルに書く
#define MPU6050_ADDRESS     0x68 // address pin AD0 low (GND)


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
		_last_us(0),
		_dt_sec(0.0f),
		_i2c(i2c),
		_healthy(false)
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
	
	bool calib_ok() const {return (gyro_calib_ok() && acc_calib_ok());}
	bool gyro_calib_ok() const {return calib_OK_G;}
	bool acc_calib_ok() const {return calib_OK_A;}
	
	float get_delta_time() const {return _dt_sec;}
	const Vector3f &get_accel() const { return _accel; }
	const Vector3f &get_gyro()  const { return _gyro; }
	bool get_gyro_health() const {return _healthy;}
	
	
private:
	int16_t		gyroZero[3];
	int16_t		accZero[3];
	uint16_t	calibratingG;	// gyro
	uint16_t	calibratingA;	// the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
	bool		calib_OK_G;
	bool		calib_OK_A;
	uint32_t	_last_us;		// 前回データ取得時刻[us]
	float		_dt_sec;		// 前回データ取得から今回取得までの時間[sec]
	Vector3f	_accel;			// 加速度値[m/s^2]
	Vector3f	_gyro;			// ジャイロ値[rad/s]
	bool		_healthy;		// センサの正常度（正常ならtrue、異常ならfalse）
	
	static const float	_gyro_scale;
	
	void gyro_common();
	void acc_common();
	//void device_mag_getADC();
	
protected:
	BC_I2C	&_i2c;
};

#endif // __BC_INERTIAL_SENSOR_H__
