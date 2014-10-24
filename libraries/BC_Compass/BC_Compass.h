/** charset=UTF-8 **/

#ifndef __BC_COMPASS_H__
#define __BC_COMPASS_H__

#include <BC_Common.h>
#include <BC_I2C.h>
#include <BC_InertialSensor.h>

// ***********************************************************************************
// defines
// ***********************************************************************************
//#define MAG_DEC    -72		// MAG DECLINATION変数、ほぼオリジナル
							// ここで入手 http://magnetic-declination.com/
							// 度分を「度+分/60」で計算 ex.)7°14′→7.23
							// 符号として、POSITIVEなら"+"をNEGATIVEなら"-"をつける
							// 最後に10倍する


// ***********************************************************************************
// class
// ***********************************************************************************
class BC_Compass
{
public:
	BC_Compass(BC_I2C &i2c):
		magInit(false),
		calibrate_mag(false),
		calib_OK_M(false),
		_healthy(false),
		_i2c(i2c)
	{}
	
	uint8_t		rawADC[6];
	int16_t		magADC[3];
	
	void		init();
	bool		get_data();
	void		calib_start();
	bool		calib_ok() const {return calib_OK_M;}
	bool		use_for_yaw() const {return _healthy;}	// 健常度を返す
	
private:
	int16_t		magZero[3];
	float		magGain[3];  // gain for each axis, populated at sensor init (デフォルトは全て1.0)
	bool		magInit;
	bool		calibrate_mag;
	int16_t		mag_declination;
	bool		calib_OK_M;
	bool		_healthy;
	
	void		getADC();
	void		getADC_via_MPU6050();
	
protected:
	BC_I2C	&_i2c;
};

#endif // __BC_INERTIAL_SENSOR_H__
