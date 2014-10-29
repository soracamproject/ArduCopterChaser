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

#include "BC_Compass.h"
#include <Arduino.h>

// ***********************************************************************************
// defines
// ***********************************************************************************
#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_X_SELF_TEST_GAUSS (+1.16)                       //!< X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16)   //!< Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08)                       //!< Y axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT  (243.0/390.0)   //!< Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0/390.0)   //!< High limit when gain is 5.
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

#define MAG_ADDRESS 0x1E	// I2C adress: 0x3C (8bit)   0x1E (7bit)
#define MAG_DATA_REGISTER 0x03
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;} 


// ***********************************************************************************
// functions
// ***********************************************************************************

void BC_Compass::init() {
	int32_t xyz_total[3]={0,0,0};  // 32 bit totals so they won't overflow.
	bool bret=true;                // Error indicator
	magGain[0]=1.0f;
	magGain[1]=1.0f;
	magGain[2]=1.0f;
	
	// MPU6050をバイパスする
	_i2c.writeReg(MPU6050_ADDRESS, 0x37, 0x02);	// INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=1 ; CLKOUT_EN=0
	
	delay(50);  //Wait before start
	_i2c.writeReg(MAG_ADDRESS, HMC58X3_R_CONFA, 0x010 + HMC_POS_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to pos bias
	
	// Note that the  very first measurement after a gain change maintains the same gain as the previous setting. 
	// The new gain setting is effective from the second measurement and on.
	
	_i2c.writeReg(MAG_ADDRESS, HMC58X3_R_CONFB, 2 << 5);  //Set the Gain
	_i2c.writeReg(MAG_ADDRESS,HMC58X3_R_MODE, 1);
	delay(100);
	getADC();  //Get one sample, and discard it
	
	for (uint8_t i=0; i<10; i++) { //Collect 10 samples
		_i2c.writeReg(MAG_ADDRESS,HMC58X3_R_MODE, 1);
		delay(100);
		getADC();   // Get the raw values in case the scales have already been changed.
		
		// Since the measurements are noisy, they should be averaged rather than taking the max.
		xyz_total[0]+=magADC[0];
		xyz_total[1]+=magADC[1];
		xyz_total[2]+=magADC[2];
		
		// Detect saturation.
		if (-(1<<12) >= min(magADC[0],min(magADC[1],magADC[2]))) {
			bret=false;
			break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
		}
	}
	
	// Apply the negative bias. (Same gain)
	_i2c.writeReg(MAG_ADDRESS,HMC58X3_R_CONFA, 0x010 + HMC_NEG_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to negative bias.
	for (uint8_t i=0; i<10; i++) { 
		_i2c.writeReg(MAG_ADDRESS,HMC58X3_R_MODE, 1);
		delay(100);
		getADC();  // Get the raw values in case the scales have already been changed.
		
		// Since the measurements are noisy, they should be averaged.
		xyz_total[0]-=magADC[0];
		xyz_total[1]-=magADC[1];
		xyz_total[2]-=magADC[2];
		
		// Detect saturation.
		if (-(1<<12) >= min(magADC[0],min(magADC[1],magADC[2]))) {
			bret=false;
			break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
		}
	}
	
	magGain[0]=fabs(820.0*HMC58X3_X_SELF_TEST_GAUSS*2.0*10.0/xyz_total[0]);
	magGain[1]=fabs(820.0*HMC58X3_Y_SELF_TEST_GAUSS*2.0*10.0/xyz_total[1]);
	magGain[2]=fabs(820.0*HMC58X3_Z_SELF_TEST_GAUSS*2.0*10.0/xyz_total[2]);
	
	// leave test mode
	_i2c.writeReg(MAG_ADDRESS ,HMC58X3_R_CONFA ,0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
	_i2c.writeReg(MAG_ADDRESS ,HMC58X3_R_CONFB ,0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
	_i2c.writeReg(MAG_ADDRESS ,HMC58X3_R_MODE  ,0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode
	delay(100);
	magInit = true;
	
	if (!bret) { //Something went wrong so get a best guess
		magGain[0] = 1.0;
		magGain[1] = 1.0;
		magGain[2] = 1.0;
	}
	
	// MPU6050のバイパスを解除しマスタ化する
	_i2c.writeReg(MPU6050_ADDRESS, 0x6A, 0b00100000);       //USER_CTRL     -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
	_i2c.writeReg(MPU6050_ADDRESS, 0x37, 0x00);             //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
	_i2c.writeReg(MPU6050_ADDRESS, 0x24, 0x0D);             //I2C_MST_CTRL  -- MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
	_i2c.writeReg(MPU6050_ADDRESS, 0x25, 0x80|MAG_ADDRESS);//I2C_SLV0_ADDR -- I2C_SLV4_RW=1 (read operation) ; I2C_SLV4_ADDR=MAG_ADDRESS
	_i2c.writeReg(MPU6050_ADDRESS, 0x26, MAG_DATA_REGISTER);//I2C_SLV0_REG  -- 6 data bytes of MAG are stored in 6 registers. First register address is MAG_DATA_REGISTER
	_i2c.writeReg(MPU6050_ADDRESS, 0x27, 0x86);             //I2C_SLV0_CTRL -- I2C_SLV0_EN=1 ; I2C_SLV0_BYTE_SW=0 ; I2C_SLV0_REG_DIS=0 ; I2C_SLV0_GRP=0 ; I2C_SLV0_LEN=3 (3x2 bytes)
}


bool BC_Compass::read(){ // return true when news values are available, false otherwise
	static uint32_t t,tCal = 0;
	static int16_t magZeroTempMin[3];
	static int16_t magZeroTempMax[3];
	uint8_t axis;
	uint32_t now_us = micros();
	if(now_us<t){
		return false; //each read is spaced by 100ms
	}
	_last_update = now_us; // record time of update
	t = now_us + 100000;
	TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
	getADC_via_MPU6050();
	magADC[ROLL]  = magADC[ROLL]  * magGain[ROLL];
	magADC[PITCH] = magADC[PITCH] * magGain[PITCH];
	magADC[YAW]   = magADC[YAW]   * magGain[YAW];
	if (calibrate_mag) {
		tCal = t;
		for(axis=0;axis<3;axis++) {
			magZero[axis] = 0;
			magZeroTempMin[axis] = magADC[axis];
			magZeroTempMax[axis] = magADC[axis];
		}
		calibrate_mag = false;
	}
	if (magInit) { // we apply offset only once mag calibration is done
		magADC[ROLL]  -= magZero[ROLL];
		magADC[PITCH] -= magZero[PITCH];
		magADC[YAW]   -= magZero[YAW];
	}
	
	if (tCal != 0) {
		if ((t - tCal) < 30000000) { // 30s: you have 30s to turn the multi in all directions
			for(axis=0;axis<3;axis++) {
				if (magADC[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = magADC[axis];
				if (magADC[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = magADC[axis];
			}
		} else {
			tCal = 0;
			for(axis=0;axis<3;axis++){
				magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])>>1;
			}
			calib_OK_M = true;
		}
	}
	return true;
}


void BC_Compass::getADC() {
	_i2c.getSixRawADC(MAG_ADDRESS,MAG_DATA_REGISTER, rawADC);
	MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
					 ((rawADC[4]<<8) | rawADC[5]) ,
					 ((rawADC[2]<<8) | rawADC[3]) );
}


// MPU6050をマスタとしてHMC5883Cを読みに行く
void BC_Compass::getADC_via_MPU6050() {
	_i2c.getSixRawADC(MPU6050_ADDRESS, 0x49, rawADC);               //0x49 is the first memory room for EXT_SENS_DATA
	MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
					 ((rawADC[4]<<8) | rawADC[5]) ,
					 ((rawADC[2]<<8) | rawADC[3]) );
}

void BC_Compass::calib_start(){
	calibrate_mag = true;
	calib_OK_M = false;
	return;
}
