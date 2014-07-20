// ***********************************************************************************
// IMU関連define
// ***********************************************************************************
#define ACC_LPF_FACTOR 4	// that means a LPF of 16
							// Set the Low Pass Filter factor for ACC
							// Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time
							// Comment this if  you do not want filter at all.
							// unit = n power of 2
#define GYR_CMPF_FACTOR 600	// Set the Gyro Weight for Gyro/Acc complementary filter
							// Increasing this value would reduce and delay Acc influence on the output of the filter
#define GYR_CMPFM_FACTOR 250	// Set the Gyro Weight for Gyro/Magnetometer complementary filter
								// Increasing this value would reduce and delay Magnetometer influence on the output of the filter
#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))


// ***********************************************************************************
// IMU関連変数、構造体
// ***********************************************************************************
typedef struct fp_vector {		
	float X,Y,Z;		
} t_fp_vector_def;

typedef union {		
	float A[3];		
	t_fp_vector_def V;		
} t_fp_vector;

typedef struct int32_t_vector {
  int32_t X,Y,Z;
} t_int32_t_vector_def;

typedef union {
  int32_t A[3];
  t_int32_t_vector_def V;
} t_int32_t_vector;

static int32_t accLPF32[3]    = {0, 0, 1};
static float invG; // 1/|G|
static t_fp_vector EstG;
static t_int32_t_vector EstG32;
static t_fp_vector EstM;
static t_int32_t_vector EstM32;



// ***********************************************************************************
// IMU関連関数
// ***********************************************************************************
void computeIMU() {
	uint8_t axis;
	static int16_t gyroADCprevious[3] = {0,0,0};
	int16_t gyroADCp[3];
	int16_t gyroADCinter[3];
	static uint32_t timeInterleave = 0;
	
	//we separate the 2 situations because reading gyro values with a gyro only setup can be acchieved at a higher rate
	//gyro+nunchuk: we must wait for a quite high delay betwwen 2 reads to get both WM+ and Nunchuk data. It works with 3ms
	//gyro only: the delay to read 2 consecutive values can be reduced to only 0.65ms
	acc_getADC();
	getEstimatedAttitude();
	gyro_getADC();
	for (axis = 0; axis < 3; axis++)
		gyroADCp[axis] =  gyroADC[axis];
	timeInterleave=micros();
	//annexCode();
	uint8_t t=0;
	while((uint16_t)(micros()-timeInterleave)<650) t=1; //empirical, interleaving delay between 2 consecutive reads	// 650μs待つ。たぶんセンサ取得周期2回分とか？
	//if (!t) annex650_overrun_count++;	// 650μs以上かかったらt=1にならないのでエラーカウントされる
	gyro_getADC();
	for (axis = 0; axis < 3; axis++) {
		gyroADCinter[axis] =  gyroADC[axis]+gyroADCp[axis];
		// empirical, we take a weighted value of the current and the previous values
		gyroData[axis] = (gyroADCinter[axis]+gyroADCprevious[axis])/3;
 		gyroADCprevious[axis] = gyroADCinter[axis]>>1;
	}
	//#if defined(GYRO_SMOOTHING)
	//static int16_t gyroSmooth[3] = {0,0,0};
	//for (axis = 0; axis < 3; axis++) {
	//	imu.gyroData[axis] = (int16_t) ( ( (int32_t)((int32_t)gyroSmooth[axis] * (conf.Smoothing[axis]-1) )+imu.gyroData[axis]+1 ) / conf.Smoothing[axis]);
	//	gyroSmooth[axis] = imu.gyroData[axis];
	//}
	//#elif defined(TRI)
	//static int16_t gyroYawSmooth = 0;
	//imu.gyroData[YAW] = (gyroYawSmooth*2+imu.gyroData[YAW])/3;
	//gyroYawSmooth = imu.gyroData[YAW];
	//#endif
}

void getEstimatedAttitude(){
	uint8_t axis;
	int32_t accMag = 0;
	float scale, deltaGyroAngle[3];
	uint8_t validAcc;
	static uint16_t previousT;
	uint16_t currentT = micros();
	
	scale = (currentT - previousT) * GYRO_SCALE; // GYRO_SCALE unit: radian/microsecond
	previousT = currentT;
	
	// Initialization
	for (axis = 0; axis < 3; axis++) {
		deltaGyroAngle[axis] = gyroADC[axis]  * scale; // radian
		
		accLPF32[axis]    -= accLPF32[axis]>>ACC_LPF_FACTOR;
		accLPF32[axis]    += accADC[axis];
		accSmooth[axis]    = accLPF32[axis]>>ACC_LPF_FACTOR;
		
		accMag += (int32_t)accSmooth[axis]*accSmooth[axis] ;
	}
	
	rotateV(&EstG.V,deltaGyroAngle);
	rotateV(&EstM.V,deltaGyroAngle);
	
	accMag = accMag*100/((int32_t)ACC_1G*ACC_1G);
	validAcc = 72 < (uint16_t)accMag && (uint16_t)accMag < 133;
	// Apply complimentary filter (Gyro drift correction)
	// If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
	// To do that, we just skip filter, as EstV already rotated by Gyro
	for (axis = 0; axis < 3; axis++) {
		if ( validAcc )
			EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
		EstG32.A[axis] = EstG.A[axis]; //int32_t cross calculation is a little bit faster than float	
		EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR  + magADC[axis]) * INV_GYR_CMPFM_FACTOR;
		EstM32.A[axis] = EstM.A[axis];
	}
	
	//if ((int16_t)EstG32.A[2] > ACCZ_25deg)
	//	f.SMALL_ANGLES_25 = 1;
	//else
	//	f.SMALL_ANGLES_25 = 0;
	
	// Attitude of the estimated vector
	int32_t sqGX_sqGZ = sq(EstG32.V.X) + sq(EstG32.V.Z);
	invG = InvSqrt(sqGX_sqGZ + sq(EstG32.V.Y));
	angle[ROLL]  = _atan2(EstG32.V.X , EstG32.V.Z);
	angle[PITCH] = _atan2(EstG32.V.Y , InvSqrt(sqGX_sqGZ)*sqGX_sqGZ);
	
	heading = _atan2(
		EstM32.V.Z * EstG32.V.X - EstM32.V.X * EstG32.V.Z,
		(EstM.V.Y * sqGX_sqGZ  - (EstM32.V.X * EstG32.V.X + EstM32.V.Z * EstG32.V.Z) * EstG.V.Y)*invG ); 
	heading += mag_declination; // Set from GUI
	heading /= 10;
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v,float* delta) {
	fp_vector v_tmp = *v;
	v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
	v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
	v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X;
}

float InvSqrt(float x){
	union{
		int32_t i;
		float   f;
	} conv;
	conv.f = x;
	conv.i = 0x5f3759df - (conv.i >> 1); 
	return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}

int16_t _atan2(int32_t y, int32_t x){
	float z = (float)y / x;
	int16_t a;
	if( abs(y) < abs(x) ){
		a = 573 * z / (1.0f + 0.28f * z * z);
		if (x<0) {
			if (y<0) a -= 1800;
			else a += 1800;
		}
	} else {
		a = 900 - 573 * z / (z * z + 0.28f);
		if (y<0) a -= 1800;
	}
	return a;
}


