/** charset=UTF-8 **/

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "BC_InertialSensor.h"
#include <Arduino.h>

// ***********************************************************************************
// defines
// ***********************************************************************************
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

#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;} 
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;} 

#define ACC_1G     512
#define ACC_SCALE  0.0191536f	// 加速度センサのスケール、m/s^2に変換
								// 計算式：GRAVITY_MSS(9.80665) / ACC_1G(512)

const float BC_InertialSensor::_gyro_scale = (0.0174532f / 16.4f);	// ジャイロセンサのスケール。LSBが16.4deg/s。[rad/s]に変換。



// ***********************************************************************************
// functions
// ***********************************************************************************
void BC_InertialSensor::gyro_common() {
	static int16_t previousGyroADC[3] = {0,0,0};
	static int32_t g[3];
	uint8_t axis=0;
	uint8_t tilt=0;
	
	if (calibratingG>0) {
		for (axis = 0; axis < 3; axis++) {
			// Reset g[axis] at start of calibration
			if (calibratingG == 512) {
				g[axis]=0;
			}
			// Sum up 512 readings
			g[axis] +=gyroADC[axis];
			// Clear global variables for next reading
			gyroADC[axis]=0;
			gyroZero[axis]=0;
			if (calibratingG == 1) {
				gyroZero[axis]=(g[axis]+256)>>9;
			}
		}
		if(calibratingG == 1){
			calib_OK_G = true;
		}
		calibratingG--;
	}
	
	for (axis = 0; axis < 3; axis++) {
		gyroADC[axis]  -= gyroZero[axis];
		//anti gyro glitch, limit the variation between two consecutive readings
		gyroADC[axis] = constrain(gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);
		previousGyroADC[axis] = gyroADC[axis];
	}
	
	// rad/sに変換
	_gyro.x = gyroADC[ROLL] *_gyro_scale;
	_gyro.y = gyroADC[PITCH]*_gyro_scale;
	_gyro.z = gyroADC[YAW]  *_gyro_scale;

}

void BC_InertialSensor::acc_common() {
	static int32_t a[3];
	if (calibratingA>0) {
		for (uint8_t axis = 0; axis < 3; axis++) {
			// Reset a[axis] at start of calibration
			if (calibratingA == 512) a[axis]=0;
			// Sum up 512 readings
			a[axis] +=accADC[axis];
			// Clear global variables for next reading
			accADC[axis]=0;
			accZero[axis]=0;
		}
		// Calculate average, shift Z down by ACC_1G and store values in EEPROM at end of calibration
		if (calibratingA == 1) {
			accZero[ROLL]  = (a[ROLL]+256)>>9;
			accZero[PITCH] = (a[PITCH]+256)>>9;
			accZero[YAW]   = ((a[YAW]+256)>>9)-ACC_1G; // for nunchuk 200=1G
			
			calib_OK_A = true;
		}
		calibratingA--;
	}
	accADC[ROLL]  -=  accZero[ROLL];
	accADC[PITCH] -=  accZero[PITCH];
	accADC[YAW]   -=  accZero[YAW];
	
	// m/s^2に変換
	_accel.x = accADC[ROLL] *ACC_SCALE;
	_accel.y = accADC[PITCH]*ACC_SCALE;
	_accel.z = accADC[YAW]  *ACC_SCALE;
}


void BC_InertialSensor::gyro_init() {
	TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
	_i2c.writeReg(MPU6050_ADDRESS, 0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
	delay(5);
	_i2c.writeReg(MPU6050_ADDRESS, 0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
	_i2c.writeReg(MPU6050_ADDRESS, 0x1A, MPU6050_DLPF_CFG); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
	_i2c.writeReg(MPU6050_ADDRESS, 0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
}

void BC_InertialSensor::gyro_getADC () {
	_i2c.getSixRawADC(MPU6050_ADDRESS, 0x43, rawADC);
	GYRO_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])>>2 , // range: +/- 8192; +/- 2000 deg/sec
					  ((rawADC[2]<<8) | rawADC[3])>>2 ,
					  ((rawADC[4]<<8) | rawADC[5])>>2 );
	gyro_common();
}


void BC_InertialSensor::acc_init () {
	_i2c.writeReg(MPU6050_ADDRESS, 0x1C, 0x10);             //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
	//note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
	//confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480
}


void BC_InertialSensor::acc_getADC () {
	_i2c.getSixRawADC(MPU6050_ADDRESS, 0x3B, rawADC);
	ACC_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])>>3 ,
					 ((rawADC[2]<<8) | rawADC[3])>>3 ,
					 ((rawADC[4]<<8) | rawADC[5])>>3 );
	acc_common();
}

void BC_InertialSensor::gyro_calib_start(){
	calibratingG = 512;
	return;
}

void BC_InertialSensor::acc_calib_start(){
	calibratingA = 512;
	return;
}

void BC_InertialSensor::init(){
	gyro_init();
	acc_init();
	
	// データ取得時間初期化
	_last_us = micros();
}

void BC_InertialSensor::get_data(){
	// データ取得時間更新
	uint32_t now_us = micros();
	_dt_sec = (now_us - _last_us) * 1.0e-6f;
	_last_us = now_us;
	
	// データ取得
	gyro_getADC();
	acc_getADC();
}

void BC_InertialSensor::calib_start(){
	gyro_calib_start();
	acc_calib_start();
}


