// ******************************************************
// ***** ビーコン用コンパス操作用プログラム *****
// ******************************************************
// module: HMC5883


// ***********************************************************************************
// compass関連define
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



// ***********************************************************************************
// compass関連変数
// ***********************************************************************************
int16_t magZero[3];
int16_t magADC[3];
static float   magGain[3] = {1.0,1.0,1.0};  // gain for each axis, populated at sensor init
static uint8_t magInit = 0;
int16_t mag_declination = -72;	// MAG DECLINATION変数、ほぼオリジナル
								// ここで入手 http://magnetic-declination.com/
								// 度分を「度+分/60」で計算 ex.)7°14′→7.23
								// 符号として、POSITIVEなら"+"をNEGATIVEなら"-"をつける
								// 最後に10倍する


// ***********************************************************************************
// compass関連関数
// ***********************************************************************************
/*
uint8_t mag_getADC() { // return 1 when news values are available, 0 otherwise
	static uint32_t t,tCal = 0;
	static int16_t magZeroTempMin[3];
	static int16_t magZeroTempMax[3];
	uint8_t axis;
	if ( now_us < t ) return 0; //each read is spaced by 100ms
	t = now_us + 100000;
	TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
	device_mag_getADC();
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
		calibrate_mag = 0;
	}
	if (magInit) { // we apply offset only once mag calibration is done
		magADC[ROLL]  -= magZero[ROLL];
		magADC[PITCH] -= magZero[PITCH];
		magADC[YAW]   -= magZero[YAW];
	}
	
	if (tCal != 0) {
		if ((t - tCal) < 30000000) { // 30s: you have 30s to turn the multi in all directions
			//LEDPIN_TOGGLE;
			for(axis=0;axis<3;axis++) {
				if (magADC[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = magADC[axis];
				if (magADC[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = magADC[axis];
			}
			blink_led(0,0,0,1);
		} else {
			tCal = 0;
			for(axis=0;axis<3;axis++) 
				magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])>>1;
			//writeGlobalSet(1);
			calibOK_M = 1;
			control_led(0,0,0,1);
		}
	}
	return 1;
}
*/

void mag_init() {
	int32_t xyz_total[3]={0,0,0};  // 32 bit totals so they won't overflow.
	bool bret=true;                // Error indicator
	
	delay(50);  //Wait before start
	i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFA, 0x010 + HMC_POS_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to pos bias
	
	// Note that the  very first measurement after a gain change maintains the same gain as the previous setting. 
	// The new gain setting is effective from the second measurement and on.
	
	i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFB, 2 << 5);  //Set the Gain
	i2c_writeReg(MAG_ADDRESS,HMC58X3_R_MODE, 1);
	delay(100);
	getADC();  //Get one sample, and discard it
	
	for (uint8_t i=0; i<10; i++) { //Collect 10 samples
		i2c_writeReg(MAG_ADDRESS,HMC58X3_R_MODE, 1);
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
	i2c_writeReg(MAG_ADDRESS,HMC58X3_R_CONFA, 0x010 + HMC_NEG_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to negative bias.
	for (uint8_t i=0; i<10; i++) { 
		i2c_writeReg(MAG_ADDRESS,HMC58X3_R_MODE, 1);
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
	i2c_writeReg(MAG_ADDRESS ,HMC58X3_R_CONFA ,0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
	i2c_writeReg(MAG_ADDRESS ,HMC58X3_R_CONFB ,0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
	i2c_writeReg(MAG_ADDRESS ,HMC58X3_R_MODE  ,0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode
	delay(100);
	magInit = 1;
	
	if (!bret) { //Something went wrong so get a best guess
		magGain[0] = 1.0;
		magGain[1] = 1.0;
		magGain[2] = 1.0;
	}
}


void getADC() {
	i2c_getSixRawADC(MAG_ADDRESS,MAG_DATA_REGISTER);
	MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
					 ((rawADC[4]<<8) | rawADC[5]) ,
					 ((rawADC[2]<<8) | rawADC[3]) );
}
