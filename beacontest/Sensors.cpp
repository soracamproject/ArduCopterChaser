#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "beacontest.h"
#include "Alarms.h"
#include "EEPROM.h"
#include "IMU.h"
#include "LCD.h"

void i2c_BMP085_UT_Start(void);

void waitTransmissionI2C();
void i2c_MS561101BA_UT_Start();
#if BARO
void Baro_Common();
#endif
void Device_Mag_getADC();
void getADC();
void Baro_init();
void Mag_init();
void ACC_init();

// ************************************************************************************************************
// board orientation and setup
// ************************************************************************************************************
//default board orientation
#if !defined(ACC_ORIENTATION) 
  #define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = X; imu.accADC[PITCH]  = Y; imu.accADC[YAW]  = Z;}
#endif
#if !defined(GYRO_ORIENTATION) 
  #define GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] = X; imu.gyroADC[PITCH] = Y; imu.gyroADC[YAW] = Z;}
#endif
#if !defined(MAG_ORIENTATION) 
  #define MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL]  = X; imu.magADC[PITCH]  = Y; imu.magADC[YAW]  = Z;}
#endif

/*** I2C address ***/
#if !defined(MPU6050_ADDRESS)
  #define MPU6050_ADDRESS     0x68 // address pin AD0 low (GND), default for FreeIMU v0.4 and InvenSense evaluation board
  //#define MPU6050_ADDRESS     0x69 // address pin AD0 high (VCC)
#endif

#if !defined(MS561101BA_ADDRESS) 
  #define MS561101BA_ADDRESS 0x77 //CBR=0 0xEE I2C address when pin CSB is connected to LOW (GND)
  //#define MS561101BA_ADDRESS 0x76 //CBR=1 0xEC I2C address when pin CSB is connected to HIGH (VCC)
#endif

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

uint8_t rawADC[6];
static uint32_t neutralizeTime = 0;
  
// ************************************************************************************************************
// I2C general functions
// ************************************************************************************************************

void i2c_init(void) {
  #if defined(INTERNAL_I2C_PULLUPS)
    I2C_PULLUPS_ENABLE
  #else
    I2C_PULLUPS_DISABLE
  #endif
  TWSR = 0;                                    // no prescaler => prescaler = 1
  TWBR = ((F_CPU / I2C_SPEED) - 16) / 2;       // change the I2C clock rate
  TWCR = 1<<TWEN;                              // enable twi module, no interrupt
}

void i2c_rep_start(uint8_t address) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) ; // send REPEAT START condition
  waitTransmissionI2C();                       // wait until transmission completed
  TWDR = address;                              // send device address
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();                       // wail until transmission completed
}

void i2c_stop(void) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  //  while(TWCR & (1<<TWSTO));                // <- can produce a blocking state with some WMP clones
}

void i2c_write(uint8_t data ) {
  TWDR = data;                                 // send data to the previously addressed device
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();
}

uint8_t i2c_read(uint8_t ack) {
  TWCR = (1<<TWINT) | (1<<TWEN) | (ack? (1<<TWEA) : 0);
  waitTransmissionI2C();
  uint8_t r = TWDR;
  if (!ack) i2c_stop();
  return r;
}

uint8_t i2c_readAck() {
  return i2c_read(1);
}

uint8_t i2c_readNak(void) {
  return i2c_read(0);
}

void waitTransmissionI2C() {
  uint16_t count = 255;
  while (!(TWCR & (1<<TWINT))) {
    count--;
    if (count==0) {              //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay
      i2c_errors_count++;
      break;
    }
  }
}

size_t i2c_read_to_buf(uint8_t add, void *buf, size_t size) {
  i2c_rep_start((add<<1) | 1);  // I2C read direction
  size_t bytes_read = 0;
  uint8_t *b = (uint8_t*)buf;
  while (size--) {
    /* acknowledge all but the final byte */
    *b++ = i2c_read(size > 0);
    /* TODO catch I2C errors here and abort */
    bytes_read++;
  }
  return bytes_read;
}

size_t i2c_read_reg_to_buf(uint8_t add, uint8_t reg, void *buf, size_t size) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  return i2c_read_to_buf(add, buf, size);
}

/* transform a series of bytes from big endian to little
   endian and vice versa. */
void swap_endianness(void *buf, size_t size) {
  /* we swap in-place, so we only have to
  * place _one_ element on a temporary tray
  */
  uint8_t tray;
  uint8_t *from;
  uint8_t *to;
  /* keep swapping until the pointers have assed each other */
  for (from = (uint8_t*)buf, to = &from[size-1]; from < to; from++, to--) {
    tray = *from;
    *from = *to;
    *to = tray;
  }
}

void i2c_getSixRawADC(uint8_t add, uint8_t reg) {
  i2c_read_reg_to_buf(add, reg, &rawADC, 6);
}

void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  i2c_write(val);        // value to write in register
  i2c_stop();
}

uint8_t i2c_readReg(uint8_t add, uint8_t reg) {
  uint8_t val;
  i2c_read_reg_to_buf(add, reg, &val, 1);
  return val;
}

// ****************
// GYRO common part
// ****************
void GYRO_Common() {
  static int16_t previousGyroADC[3] = {0,0,0};
  static int32_t g[3];
  uint8_t axis, tilt=0;

#if defined MMGYRO       
  // Moving Average Gyros by Magnetron1
  //---------------------------------------------------
  static int16_t mediaMobileGyroADC[3][MMGYROVECTORLENGTH];
  static int32_t mediaMobileGyroADCSum[3];
  static uint8_t mediaMobileGyroIDX;
  //---------------------------------------------------
#endif

  if (calibratingG>0) {
    for (axis = 0; axis < 3; axis++) {
      // Reset g[axis] at start of calibration
      if (calibratingG == 512) {
        g[axis]=0;
        
        #if defined(GYROCALIBRATIONFAILSAFE)
            previousGyroADC[axis] = imu.gyroADC[axis];
          }
          if (calibratingG % 10 == 0) {
            if(abs(imu.gyroADC[axis] - previousGyroADC[axis]) > 8) tilt=1;
            previousGyroADC[axis] = imu.gyroADC[axis];
       #endif
      }
      // Sum up 512 readings
      g[axis] +=imu.gyroADC[axis];
      // Clear global variables for next reading
      imu.gyroADC[axis]=0;
      gyroZero[axis]=0;
      if (calibratingG == 1) {
        gyroZero[axis]=(g[axis]+256)>>9;
      #if defined(BUZZER)
        alarmArray[7] = 4;
      #else
        blinkLED(10,15,1); //the delay causes to beep the buzzer really long 
      #endif
      }
    }
    #if defined(GYROCALIBRATIONFAILSAFE)
      if(tilt) {
        calibratingG=1000;
        LEDPIN_ON;
      } else {
        calibratingG--;
        LEDPIN_OFF;
      }
      return;
    #else
      calibratingG--;
    #endif
    
  }

#ifdef MMGYRO       
  mediaMobileGyroIDX = ++mediaMobileGyroIDX % conf.mmgyro;
  for (axis = 0; axis < 3; axis++) {
    imu.gyroADC[axis]  -= gyroZero[axis];
    mediaMobileGyroADCSum[axis] -= mediaMobileGyroADC[axis][mediaMobileGyroIDX];
    //anti gyro glitch, limit the variation between two consecutive readings
    mediaMobileGyroADC[axis][mediaMobileGyroIDX] = constrain(imu.gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);
    mediaMobileGyroADCSum[axis] += mediaMobileGyroADC[axis][mediaMobileGyroIDX];
    imu.gyroADC[axis] = mediaMobileGyroADCSum[axis] / conf.mmgyro;
#else 
  for (axis = 0; axis < 3; axis++) {
    imu.gyroADC[axis]  -= gyroZero[axis];
    //anti gyro glitch, limit the variation between two consecutive readings
    imu.gyroADC[axis] = constrain(imu.gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);
#endif    
    previousGyroADC[axis] = imu.gyroADC[axis];
  }

  #if defined(SENSORS_TILT_45DEG_LEFT)
    int16_t temp  = ((imu.gyroADC[PITCH] - imu.gyroADC[ROLL] )*7)/10;
    imu.gyroADC[ROLL] = ((imu.gyroADC[ROLL]  + imu.gyroADC[PITCH])*7)/10;
    imu.gyroADC[PITCH]= temp;
  #endif
  #if defined(SENSORS_TILT_45DEG_RIGHT)
    int16_t temp  = ((imu.gyroADC[PITCH] + imu.gyroADC[ROLL] )*7)/10;
    imu.gyroADC[ROLL] = ((imu.gyroADC[ROLL]  - imu.gyroADC[PITCH])*7)/10;
    imu.gyroADC[PITCH]= temp;
  #endif
}

// ****************
// ACC common part
// ****************
void ACC_Common() {
  static int32_t a[3];
  if (calibratingA>0) {
    for (uint8_t axis = 0; axis < 3; axis++) {
      // Reset a[axis] at start of calibration
      if (calibratingA == 512) a[axis]=0;
      // Sum up 512 readings
      a[axis] +=imu.accADC[axis];
      // Clear global variables for next reading
      imu.accADC[axis]=0;
      global_conf.accZero[axis]=0;
    }
    // Calculate average, shift Z down by ACC_1G and store values in EEPROM at end of calibration
    if (calibratingA == 1) {
      global_conf.accZero[ROLL]  = (a[ROLL]+256)>>9;
      global_conf.accZero[PITCH] = (a[PITCH]+256)>>9;
      global_conf.accZero[YAW]   = ((a[YAW]+256)>>9)-ACC_1G; // for nunchuk 200=1G
      conf.angleTrim[ROLL]   = 0;
      conf.angleTrim[PITCH]  = 0;
      writeGlobalSet(1); // write accZero in EEPROM
    }
    calibratingA--;
  }
  #if defined(INFLIGHT_ACC_CALIBRATION)
      static int32_t b[3];
      static int16_t accZero_saved[3]  = {0,0,0};
      static int16_t  angleTrim_saved[2] = {0, 0};
      //Saving old zeropoints before measurement
      if (InflightcalibratingA==50) {
         accZero_saved[ROLL]  = global_conf.accZero[ROLL] ;
         accZero_saved[PITCH] = global_conf.accZero[PITCH];
         accZero_saved[YAW]   = global_conf.accZero[YAW] ;
         angleTrim_saved[ROLL]  = conf.angleTrim[ROLL] ;
         angleTrim_saved[PITCH] = conf.angleTrim[PITCH] ;
      }
      if (InflightcalibratingA>0) {
        for (uint8_t axis = 0; axis < 3; axis++) {
          // Reset a[axis] at start of calibration
          if (InflightcalibratingA == 50) b[axis]=0;
          // Sum up 50 readings
          b[axis] +=imu.accADC[axis];
          // Clear global variables for next reading
          imu.accADC[axis]=0;
          global_conf.accZero[axis]=0;
        }
        //all values are measured
        if (InflightcalibratingA == 1) {
          AccInflightCalibrationActive = 0;
          AccInflightCalibrationMeasurementDone = 1;
          #if defined(BUZZER)
            alarmArray[7] = 1;      //buzzer for indicatiing the end of calibration
          #endif
          // recover saved values to maintain current flight behavior until new values are transferred
          global_conf.accZero[ROLL]  = accZero_saved[ROLL] ;
          global_conf.accZero[PITCH] = accZero_saved[PITCH];
          global_conf.accZero[YAW]   = accZero_saved[YAW] ;
          conf.angleTrim[ROLL]  = angleTrim_saved[ROLL] ;
          conf.angleTrim[PITCH] = angleTrim_saved[PITCH] ;
        }
        InflightcalibratingA--;
      }
      // Calculate average, shift Z down by ACC_1G and store values in EEPROM at end of calibration
      if (AccInflightCalibrationSavetoEEProm == 1){  //the copter is landed, disarmed and the combo has been done again
        AccInflightCalibrationSavetoEEProm = 0;
        global_conf.accZero[ROLL]  = b[ROLL]/50;
        global_conf.accZero[PITCH] = b[PITCH]/50;
        global_conf.accZero[YAW]   = b[YAW]/50-ACC_1G; // for nunchuk 200=1G
        conf.angleTrim[ROLL]   = 0;
        conf.angleTrim[PITCH]  = 0;
        writeGlobalSet(1); // write accZero in EEPROM
      }
  #endif
  imu.accADC[ROLL]  -=  global_conf.accZero[ROLL] ;
  imu.accADC[PITCH] -=  global_conf.accZero[PITCH];
  imu.accADC[YAW]   -=  global_conf.accZero[YAW] ;

  #if defined(SENSORS_TILT_45DEG_LEFT)
    int16_t temp = ((imu.accADC[PITCH] - imu.accADC[ROLL] )*7)/10;
    imu.accADC[ROLL] = ((imu.accADC[ROLL]  + imu.accADC[PITCH])*7)/10;
    imu.accADC[PITCH] = temp;
  #endif
  #if defined(SENSORS_TILT_45DEG_RIGHT)
    int16_t temp = ((imu.accADC[PITCH] + imu.accADC[ROLL] )*7)/10;
    imu.accADC[ROLL] = ((imu.accADC[ROLL]  - imu.accADC[PITCH])*7)/10;
    imu.accADC[PITCH] = temp;
  #endif
}


// ************************************************************************************************************
// I2C Barometer MS561101BA
// ************************************************************************************************************
//
// specs are here: http://www.meas-spec.com/downloads/MS5611-01BA03.pdf
// useful info on pages 7 -> 12
#if defined(MS561101BA)

// registers of the device
#define MS561101BA_PRESSURE    0x40
#define MS561101BA_TEMPERATURE 0x50
#define MS561101BA_RESET       0x1E

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256  0x00
#define MS561101BA_OSR_512  0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define OSR MS561101BA_OSR_4096

static struct {
  // sensor registers from the MS561101BA datasheet
  uint16_t c[7];
  union {uint32_t val; uint8_t raw[4]; } ut; //uncompensated T
  union {uint32_t val; uint8_t raw[4]; } up; //uncompensated P
  uint8_t  state;
  uint32_t deadline;
} ms561101ba_ctx;

void i2c_MS561101BA_reset(){
  i2c_writeReg(MS561101BA_ADDRESS, MS561101BA_RESET, 0);
}

void i2c_MS561101BA_readCalibration(){
  union {uint16_t val; uint8_t raw[2]; } data;
  for(uint8_t i=0;i<6;i++) {
    i2c_rep_start(MS561101BA_ADDRESS<<1);
    i2c_write(0xA2+2*i);
    delay(10);
    i2c_rep_start((MS561101BA_ADDRESS<<1) | 1);//I2C read direction => 1
    delay(10);
    data.raw[1] = i2c_readAck();  // read a 16 bit register
    data.raw[0] = i2c_readNak();
    ms561101ba_ctx.c[i+1] = data.val;
  }
}

void  Baro_init() {
  delay(10);
  i2c_MS561101BA_reset();
  delay(100);
  i2c_MS561101BA_readCalibration();
  delay(10);
  i2c_MS561101BA_UT_Start(); 
  ms561101ba_ctx.deadline = currentTime+10000; 
}

// read uncompensated temperature value: send command first
void i2c_MS561101BA_UT_Start() {
  i2c_rep_start(MS561101BA_ADDRESS<<1);      // I2C write direction
  i2c_write(MS561101BA_TEMPERATURE + OSR);  // register selection
  i2c_stop();
}

// read uncompensated pressure value: send command first
void i2c_MS561101BA_UP_Start () {
  i2c_rep_start(MS561101BA_ADDRESS<<1);      // I2C write direction
  i2c_write(MS561101BA_PRESSURE + OSR);     // register selection
  i2c_stop();
}

// read uncompensated pressure value: read result bytes
void i2c_MS561101BA_UP_Read () {
  i2c_rep_start(MS561101BA_ADDRESS<<1);
  i2c_write(0);
  i2c_rep_start((MS561101BA_ADDRESS<<1) | 1);
  ms561101ba_ctx.up.raw[2] = i2c_readAck();
  ms561101ba_ctx.up.raw[1] = i2c_readAck();
  ms561101ba_ctx.up.raw[0] = i2c_readNak();
}

// read uncompensated temperature value: read result bytes
void i2c_MS561101BA_UT_Read() {
  i2c_rep_start(MS561101BA_ADDRESS<<1);
  i2c_write(0);
  i2c_rep_start((MS561101BA_ADDRESS<<1) | 1);
  ms561101ba_ctx.ut.raw[2] = i2c_readAck();
  ms561101ba_ctx.ut.raw[1] = i2c_readAck();
  ms561101ba_ctx.ut.raw[0] = i2c_readNak();
}

void i2c_MS561101BA_Calculate() {
  int32_t off2,sens2,delt;

  int64_t dT       = (int32_t)ms561101ba_ctx.ut.val - ((int32_t)ms561101ba_ctx.c[5] << 8);
  baroTemperature  = 2000 + ((dT * ms561101ba_ctx.c[6])>>23);
  int64_t off      = ((uint32_t)ms561101ba_ctx.c[2] <<16) + ((dT * ms561101ba_ctx.c[4]) >> 7);
  int64_t sens     = ((uint32_t)ms561101ba_ctx.c[1] <<15) + ((dT * ms561101ba_ctx.c[3]) >> 8);

  if (baroTemperature < 2000) { // temperature lower than 20st.C 
    delt = baroTemperature-2000;
    delt  = 5*delt*delt;
    off2  = delt>>1;
    sens2 = delt>>2;
    if (baroTemperature < -1500) { // temperature lower than -15st.C
      delt  = baroTemperature+1500;
      delt  = delt*delt;
      off2  += 7 * delt;
      sens2 += (11 * delt)>>1;
    }
    off  -= off2; 
    sens -= sens2;
  }

  baroPressure     = (( (ms561101ba_ctx.up.val * sens ) >> 21) - off) >> 15;
}

//return 0: no data available, no computation ;  1: new value available  ; 2: no new value, but computation time
uint8_t Baro_update() {                            // first UT conversion is started in init procedure
  if (currentTime < ms561101ba_ctx.deadline) return 0; 
  ms561101ba_ctx.deadline = currentTime+10000;  // UT and UP conversion take 8.5ms so we do next reading after 10ms 
  TWBR = ((F_CPU / 400000L) - 16) / 2;          // change the I2C clock rate to 400kHz, MS5611 is ok with this speed
  if (ms561101ba_ctx.state == 0) {
    i2c_MS561101BA_UT_Read(); 
    i2c_MS561101BA_UP_Start(); 
    Baro_Common();                              // moved here for less timecycle spike
    ms561101ba_ctx.state = 1;
    return 1;
  } else {
    i2c_MS561101BA_UP_Read();
    i2c_MS561101BA_UT_Start(); 
    i2c_MS561101BA_Calculate();
    ms561101ba_ctx.state = 0; 
    return 2;
  }
}
#endif

#if BARO
  void Baro_Common() {
    static int32_t baroHistTab[BARO_TAB_SIZE];
    static uint8_t baroHistIdx;
  
    uint8_t indexplus1 = (baroHistIdx + 1);
    if (indexplus1 == BARO_TAB_SIZE) indexplus1 = 0;
    baroHistTab[baroHistIdx] = baroPressure;
    baroPressureSum += baroHistTab[baroHistIdx];
    baroPressureSum -= baroHistTab[indexplus1];
    baroHistIdx = indexplus1;  
  }
#endif



// ************************************************************************************************************
// ADC ACC
// ************************************************************************************************************
#if defined(ADCACC)
void ACC_init(){
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
}

void ACC_getADC() {
  ACC_ORIENTATION(  analogRead(A1) ,
                    analogRead(A2) ,
                    analogRead(A3) );
  ACC_Common();
}
#endif



// ************************************************************************************************************
// I2C Compass common function
// ************************************************************************************************************
#if MAG
static float   magGain[3] = {1.0,1.0,1.0};  // gain for each axis, populated at sensor init
static uint8_t magInit = 0;

uint8_t Mag_getADC() { // return 1 when news values are available, 0 otherwise
  static uint32_t t,tCal = 0;
  static int16_t magZeroTempMin[3];
  static int16_t magZeroTempMax[3];
  uint8_t axis;
  if ( currentTime < t ) return 0; //each read is spaced by 100ms
  t = currentTime + 100000;
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  Device_Mag_getADC();
  imu.magADC[ROLL]  = imu.magADC[ROLL]  * magGain[ROLL];
  imu.magADC[PITCH] = imu.magADC[PITCH] * magGain[PITCH];
  imu.magADC[YAW]   = imu.magADC[YAW]   * magGain[YAW];
  if (f.CALIBRATE_MAG) {
    tCal = t;
    for(axis=0;axis<3;axis++) {
      global_conf.magZero[axis] = 0;
      magZeroTempMin[axis] = imu.magADC[axis];
      magZeroTempMax[axis] = imu.magADC[axis];
    }
    f.CALIBRATE_MAG = 0;
  }
  if (magInit) { // we apply offset only once mag calibration is done
    imu.magADC[ROLL]  -= global_conf.magZero[ROLL];
    imu.magADC[PITCH] -= global_conf.magZero[PITCH];
    imu.magADC[YAW]   -= global_conf.magZero[YAW];
  }
 
  if (tCal != 0) {
    if ((t - tCal) < 30000000) { // 30s: you have 30s to turn the multi in all directions
      LEDPIN_TOGGLE;
      for(axis=0;axis<3;axis++) {
        if (imu.magADC[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = imu.magADC[axis];
        if (imu.magADC[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = imu.magADC[axis];
      }
    } else {
      tCal = 0;
      for(axis=0;axis<3;axis++)
        global_conf.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])>>1;
      writeGlobalSet(1);
    }
  } else {
    #if defined(SENSORS_TILT_45DEG_LEFT)
      int16_t temp = ((imu.magADC[PITCH] - imu.magADC[ROLL] )*7)/10;
      imu.magADC[ROLL] = ((imu.magADC[ROLL]  + imu.magADC[PITCH])*7)/10;
      imu.magADC[PITCH] = temp;
    #endif
    #if defined(SENSORS_TILT_45DEG_RIGHT)
      int16_t temp = ((imu.magADC[PITCH] + imu.magADC[ROLL] )*7)/10;
      imu.magADC[ROLL] = ((imu.magADC[ROLL]  - imu.magADC[PITCH])*7)/10;
      imu.magADC[PITCH] = temp;
    #endif
  }
  return 1;
}
#endif

// ************************************************************************************************************
// I2C Compass MAG3110
// ************************************************************************************************************
// I2C adress: 0x0E (7bit)
// ************************************************************************************************************
#if defined(MAG3110)
  #define MAG_ADDRESS 0x0E
  #define MAG_DATA_REGISTER 0x01
  #define MAG_CTRL_REG1 0x10
  #define MAG_CTRL_REG2 0x11
  
  void Mag_init() {
    delay(100);
    i2c_writeReg(MAG_ADDRESS,MAG_CTRL_REG2,0x80);  //Automatic Magnetic Sensor Reset
    delay(100);
    i2c_writeReg(MAG_ADDRESS,MAG_CTRL_REG1,0x11); // DR = 20Hz ; OS ratio = 64 ; mode = Active
    delay(100);
    magInit = 1;
  }
  
  #if !defined(MPU6050_I2C_AUX_MASTER)
    void Device_Mag_getADC() {
      i2c_getSixRawADC(MAG_ADDRESS,MAG_DATA_REGISTER);
      MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,          
                       ((rawADC[2]<<8) | rawADC[3]) ,     
                       ((rawADC[4]<<8) | rawADC[5]) );
    }
  #endif
#endif
// ************************************************************************************************************
// I2C Compass HMC5883
// ************************************************************************************************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)
// ************************************************************************************************************

#if defined(HMC5883)

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

#define MAG_ADDRESS 0x1E
#define MAG_DATA_REGISTER 0x03

void Mag_init() {
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
    xyz_total[0]+=imu.magADC[0];
    xyz_total[1]+=imu.magADC[1];
    xyz_total[2]+=imu.magADC[2];
                
    // Detect saturation.
    if (-(1<<12) >= min(imu.magADC[0],min(imu.magADC[1],imu.magADC[2]))) {
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
    xyz_total[0]-=imu.magADC[0];
    xyz_total[1]-=imu.magADC[1];
    xyz_total[2]-=imu.magADC[2];

    // Detect saturation.
    if (-(1<<12) >= min(imu.magADC[0],min(imu.magADC[1],imu.magADC[2]))) {
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
} //  Mag_init().
#endif

// ************************************************************************************************************
// I2C Compass HMC5843
// ************************************************************************************************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)
// ************************************************************************************************************
#if defined(HMC5843)

  #define MAG_ADDRESS 0x1E
  #define MAG_DATA_REGISTER 0x03
  
  void Mag_init() { 
    delay(100);
    // force positiveBias
    i2c_writeReg(MAG_ADDRESS ,0x00 ,0x71 ); //Configuration Register A  -- 0 11 100 01  num samples: 8 ; output rate: 15Hz ; positive bias
    delay(50);
    // set gains for calibration
    i2c_writeReg(MAG_ADDRESS ,0x01 ,0x60 ); //Configuration Register B  -- 011 00000    configuration gain 2.5Ga
    i2c_writeReg(MAG_ADDRESS ,0x02 ,0x01 ); //Mode register             -- 000000 01    single Conversion Mode

    // read values from the compass -  self test operation
    // by placing the mode register into single-measurement mode (0x01), two data acquisition cycles will be made on each magnetic vector.
    // The first acquisition values will be subtracted from the second acquisition, and the net measurement will be placed into the data output registers
    delay(100);
      getADC();
    delay(10);
      magGain[ROLL]  =  1000.0 / abs(imu.magADC[ROLL]);
      magGain[PITCH] =  1000.0 / abs(imu.magADC[PITCH]);
      magGain[YAW]   =  1000.0 / abs(imu.magADC[YAW]);

    // leave test mode
    i2c_writeReg(MAG_ADDRESS ,0x00 ,0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
    i2c_writeReg(MAG_ADDRESS ,0x01 ,0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
    i2c_writeReg(MAG_ADDRESS ,0x02 ,0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode

    magInit = 1;
  }
#endif
  
#if defined(HMC5843) || defined(HMC5883)
void getADC() {
  i2c_getSixRawADC(MAG_ADDRESS,MAG_DATA_REGISTER);
  #if defined(HMC5843)
    MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                     ((rawADC[2]<<8) | rawADC[3]) ,
                     ((rawADC[4]<<8) | rawADC[5]) );
  #endif
  #if defined (HMC5883)  
    MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                     ((rawADC[4]<<8) | rawADC[5]) ,
                     ((rawADC[2]<<8) | rawADC[3]) );
  #endif
}

#if !defined(MPU6050_I2C_AUX_MASTER)
void Device_Mag_getADC() {
  getADC();
}
#endif

#endif

// ************************************************************************************************************
// I2C Compass AK8975
// ************************************************************************************************************
// I2C adress: 0x0C (7bit)
// ************************************************************************************************************
#if defined(AK8975)
  #define MAG_ADDRESS 0x0C
  #define MAG_DATA_REGISTER 0x03
  
  void Mag_init() {
    delay(100);
    i2c_writeReg(MAG_ADDRESS,0x0a,0x01);  //Start the first conversion
    delay(100);
    magInit = 1;
  }

  void Device_Mag_getADC() {
    i2c_getSixRawADC(MAG_ADDRESS,MAG_DATA_REGISTER);
    MAG_ORIENTATION( ((rawADC[1]<<8) | rawADC[0]) ,          
                     ((rawADC[3]<<8) | rawADC[2]) ,     
                     ((rawADC[5]<<8) | rawADC[4]) );
    //Start another meassurement
    i2c_writeReg(MAG_ADDRESS,0x0a,0x01);
  }
#endif

// ************************************************************************************************************
// I2C Gyroscope and Accelerometer MPU6050
// ************************************************************************************************************
#if defined(MPU6050)

void Gyro_init() {
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
  delay(5);
  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  i2c_writeReg(MPU6050_ADDRESS, 0x1A, MPU6050_DLPF_CFG); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  i2c_writeReg(MPU6050_ADDRESS, 0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
  // enable I2C bypass for AUX I2C
  #if defined(MAG)
    i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x02);           //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=1 ; CLKOUT_EN=0
  #endif
}

void Gyro_getADC () {
  i2c_getSixRawADC(MPU6050_ADDRESS, 0x43);
  GYRO_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])>>2 , // range: +/- 8192; +/- 2000 deg/sec
                    ((rawADC[2]<<8) | rawADC[3])>>2 ,
                    ((rawADC[4]<<8) | rawADC[5])>>2 );
  GYRO_Common();
}

void ACC_init () {
  i2c_writeReg(MPU6050_ADDRESS, 0x1C, 0x10);             //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
  //note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
  //confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480

  #if defined(MPU6050_I2C_AUX_MASTER)
    //at this stage, the MAG is configured via the original MAG init function in I2C bypass mode
    //now we configure MPU as a I2C Master device to handle the MAG via the I2C AUX port (done here for HMC5883)
    i2c_writeReg(MPU6050_ADDRESS, 0x6A, 0b00100000);       //USER_CTRL     -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
    i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x00);             //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
    i2c_writeReg(MPU6050_ADDRESS, 0x24, 0x0D);             //I2C_MST_CTRL  -- MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
    i2c_writeReg(MPU6050_ADDRESS, 0x25, 0x80|MAG_ADDRESS);//I2C_SLV0_ADDR -- I2C_SLV4_RW=1 (read operation) ; I2C_SLV4_ADDR=MAG_ADDRESS
    i2c_writeReg(MPU6050_ADDRESS, 0x26, MAG_DATA_REGISTER);//I2C_SLV0_REG  -- 6 data bytes of MAG are stored in 6 registers. First register address is MAG_DATA_REGISTER
    i2c_writeReg(MPU6050_ADDRESS, 0x27, 0x86);             //I2C_SLV0_CTRL -- I2C_SLV0_EN=1 ; I2C_SLV0_BYTE_SW=0 ; I2C_SLV0_REG_DIS=0 ; I2C_SLV0_GRP=0 ; I2C_SLV0_LEN=3 (3x2 bytes)
  #endif
}

void ACC_getADC () {
  i2c_getSixRawADC(MPU6050_ADDRESS, 0x3B);
  ACC_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])>>3 ,
                   ((rawADC[2]<<8) | rawADC[3])>>3 ,
                   ((rawADC[4]<<8) | rawADC[5])>>3 );
  ACC_Common();
}

//The MAG acquisition function must be replaced because we now talk to the MPU device
  #if defined(MPU6050_I2C_AUX_MASTER)
    void Device_Mag_getADC() {
      i2c_getSixRawADC(MPU6050_ADDRESS, 0x49);               //0x49 is the first memory room for EXT_SENS_DATA
      #if defined(HMC5843)
        MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                         ((rawADC[2]<<8) | rawADC[3]) ,
                         ((rawADC[4]<<8) | rawADC[5]) );
      #endif
      #if defined (HMC5883)  
        MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                         ((rawADC[4]<<8) | rawADC[5]) ,
                         ((rawADC[2]<<8) | rawADC[3]) );
      #endif
      #if defined (MAG3110)
        MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,          
                         ((rawADC[2]<<8) | rawADC[3]) ,     
                         ((rawADC[4]<<8) | rawADC[5]) );
      #endif
    }
  #endif
#endif

// ************************************************************************************************************
// Start Of I2C Gyroscope and Accelerometer LSM330
// ************************************************************************************************************
#if defined(LSM330)
////////////////////////////////////	
//           ACC start            //
////////////////////////////////////
void ACC_init () {

  delay(10);
  //i2c_writeReg(LSM330_ACC_ADDRESS ,0x20 ,0x17 ); // 1Hz
  //i2c_writeReg(LSM330_ACC_ADDRESS ,0x20 ,0x27 ); // 10Hz
  i2c_writeReg(LSM330_ACC_ADDRESS ,0x20 ,0x37 );  // 25Hz
  //i2c_writeReg(LSM330_ACC_ADDRESS ,0x20 ,0x47 ); // 50Hz
  //i2c_writeReg(LSM330_ACC_ADDRESS ,0x20 ,0x57 ); // 100Hz
  
  delay(5);
  //i2c_writeReg(LSM330_ACC_ADDRESS ,0x23 ,0x08 ); // 2G
  //i2c_writeReg(LSM330_ACC_ADDRESS ,0x23 ,0x18 ); // 4G
  i2c_writeReg(LSM330_ACC_ADDRESS ,0x23 ,0x28 ); // 8G
  //i2c_writeReg(LSM330_ACC_ADDRESS ,0x23 ,0x38 ); // 16G 
  
  delay(5);
  i2c_writeReg(LSM330_ACC_ADDRESS,0x21,0x00);// no high-pass filter
}

//#define ACC_DELIMITER 5 // for 2g
#define ACC_DELIMITER 4 // for 4g
//#define ACC_DELIMITER 3 // for 8g
//#define ACC_DELIMITER 2 // for 16g

  void ACC_getADC () {
  TWBR = ((F_CPU / 400000L) - 16) / 2;// change the I2C clock rate to 400kHz
  i2c_getSixRawADC(LSM330_ACC_ADDRESS,0x80|0x28);// Start multiple read at reg 0x28

  ACC_ORIENTATION( ((rawADC[1]<<8) | rawADC[0])>>ACC_DELIMITER ,
                   ((rawADC[3]<<8) | rawADC[2])>>ACC_DELIMITER ,
                   ((rawADC[5]<<8) | rawADC[4])>>ACC_DELIMITER );
  ACC_Common();
}
////////////////////////////////////
//            ACC end             //
////////////////////////////////////	
	
////////////////////////////////////	
//           Gyro start           //
////////////////////////////////////
void Gyro_init() {
  delay(100);
  i2c_writeReg(LSM330_GYRO_ADDRESS ,0x20 ,0x8F ); // CTRL_REG1   400Hz ODR, 20hz filter, run!
  delay(5);
  i2c_writeReg(LSM330_GYRO_ADDRESS ,0x24 ,0x02 ); // CTRL_REG5   low pass filter enable
  delay(5);
  i2c_writeReg(LSM330_GYRO_ADDRESS ,0x23 ,0x30); // CTRL_REG4 Select 2000dps
}

void Gyro_getADC () {
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_getSixRawADC(LSM330_GYRO_ADDRESS,0x80|0x28);

  GYRO_ORIENTATION( ((rawADC[1]<<8) | rawADC[0])>>2  ,
                    ((rawADC[3]<<8) | rawADC[2])>>2  ,
                    ((rawADC[5]<<8) | rawADC[4])>>2  );
  GYRO_Common();
}
////////////////////////////////////
//            Gyro end            //
////////////////////////////////////

#endif /* LSM330 */

// ************************************************************************************************************
// End Of I2C Gyroscope and Accelerometer LSM330
// ************************************************************************************************************

// ************************************************************************************************************
// I2C Gyroscope MPU3050
// ************************************************************************************************************
#if defined(MPU3050)

void Gyro_init() {
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_writeReg(MPU3050_ADDRESS, 0x3E, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
  delay(5);
  i2c_writeReg(MPU3050_ADDRESS, 0x3E, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  i2c_writeReg(MPU3050_ADDRESS, 0x16, MPU3050_DLPF_CFG + 0x18); // Gyro CONFIG   -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => GYRO bandwidth = 256Hz); -- FS_SEL = 3: Full scale set to 2000 deg/sec
}

void Gyro_getADC () {
  i2c_getSixRawADC(MPU3050_ADDRESS, 0x1D);
  GYRO_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])>>2 , // range: +/- 8192; +/- 2000 deg/sec
                    ((rawADC[2]<<8) | rawADC[3])>>2 ,
                    ((rawADC[4]<<8) | rawADC[5])>>2 );
  GYRO_Common();
}

#endif


#if defined(WMP) || defined(NUNCHUCK)
// ************************************************************************************************************
// I2C Wii Motion Plus + optional Nunchuk
// ************************************************************************************************************
// I2C adress 1: 0x53 (7bit)
// I2C adress 2: 0x52 (7bit)
// ************************************************************************************************************
#define WMP_ADDRESS_1 0x53
#define WMP_ADDRESS_2 0x52

void Gyro_init() {
  delay(250);
  i2c_writeReg(WMP_ADDRESS_1, 0xF0, 0x55); // Initialize Extension
  delay(250);
  i2c_writeReg(WMP_ADDRESS_1, 0xFE, 0x05); // Activate Nunchuck pass-through mode
  delay(250);
}

void Gyro_getADC() {
  uint8_t axis;
  TWBR = ((F_CPU / I2C_SPEED) - 16) / 2; // change the I2C clock rate
  i2c_getSixRawADC(WMP_ADDRESS_2,0x00);

  if (micros() < (neutralizeTime + NEUTRALIZE_DELAY)) {//we neutralize data in case of blocking+hard reset state
    for (axis = 0; axis < 3; axis++) {imu.gyroADC[axis]=0;imu.accADC[axis]=0;}
    imu.accADC[YAW] = ACC_1G;
    f.NUNCHUKDATA = 0;
  } 

  // Wii Motion Plus Data
  if ( (rawADC[5]&0x03) == 0x02 ) {
    // Assemble 14bit data 
    imu.gyroADC[ROLL]  = - ( ((rawADC[5]>>2)<<8) | rawADC[2] ); //range: +/- 8192
    imu.gyroADC[PITCH] = - ( ((rawADC[4]>>2)<<8) | rawADC[1] );
    imu.gyroADC[YAW]  =  - ( ((rawADC[3]>>2)<<8) | rawADC[0] );
    GYRO_Common();
    // Check if slow bit is set and normalize to fast mode range
    imu.gyroADC[ROLL]  = (rawADC[3]&0x01)     ? imu.gyroADC[ROLL]/5  : imu.gyroADC[ROLL];  //the ratio 1/5 is not exactly the IDG600 or ISZ650 specification 
    imu.gyroADC[PITCH] = (rawADC[4]&0x02)>>1  ? imu.gyroADC[PITCH]/5 : imu.gyroADC[PITCH]; //we detect here the slow of fast mode WMP gyros values (see wiibrew for more details)
    imu.gyroADC[YAW]   = (rawADC[3]&0x02)>>1  ? imu.gyroADC[YAW]/5   : imu.gyroADC[YAW];   // this step must be done after zero compensation    
    f.NUNCHUKDATA = 0;
  #if defined(NUNCHUCK)
    } else if ( (rawADC[5]&0x03) == 0x00 ) { // Nunchuk Data
      ACC_ORIENTATION(  ( (rawADC[3]<<2)      | ((rawADC[5]>>4)&0x02) ) ,
                      - ( (rawADC[2]<<2)      | ((rawADC[5]>>3)&0x02) ) ,
                        ( ((rawADC[4]>>1)<<3) | ((rawADC[5]>>5)&0x06) ) );
      ACC_Common();
      f.NUNCHUKDATA = 1;
  #endif
  }
}

#if defined(NUNCHUCK)
  void ACC_init () {
    // We need to set ACC_1G for the Nunchuk beforehand -> moved in def.h
    // If a different accelerometer is used, it will be overwritten by its ACC_init() later.
  }
  void ACC_getADC () { // it's done ine the WMP gyro part
    Gyro_getADC();
  }
#endif

#endif

// ************************************************************************************************************
// I2C Sonar SRF08
// ************************************************************************************************************
// first contribution from guru_florida (02-25-2012)
//
// specs are here: http://www.meas-spec.com/downloads/MS5611-01BA03.pdf
// useful info on pages 7 -> 12
#if defined(SRF02) || defined(SRF08) || defined(SRF10) || defined(SRC235)

// the default address for any new sensor found on the bus
// the code will move new sonars to the next available sonar address in range of F0-FE so that another
// sonar sensor can be added again.
// Thus, add only 1 sonar sensor at a time, poweroff, then wire the next, power on, wait for flashing light and repeat
#if !defined(SRF08_DEFAULT_ADDRESS) 
  #define SRF08_DEFAULT_ADDRESS 0x70
#endif

#if !defined(SRF08_RANGE_WAIT) 
  #define SRF08_RANGE_WAIT     80000      // delay between Ping and Range Read commands
#endif

#if !defined(SRF08_RANGE_SLEEP) 
  #define SRF08_RANGE_SLEEP    35000      // sleep this long before starting another Ping
#endif

#if !defined(SRF08_SENSOR_FIRST) 
  #define SRF08_SENSOR_FIRST    0xF0    // the first sensor i2c address (after it has been moved)
#endif

#if !defined(SRF08_MAX_SENSORS) 
  #define SRF08_MAX_SENSORS    4        // maximum number of sensors we'll allow (can go up to 8)
#endif

#define SONAR_MULTICAST_PING

// registers of the device
#define SRF08_REV_COMMAND    0
#define SRF08_LIGHT_GAIN     1
#define SRF08_ECHO_RANGE     2


static struct {
  // sensor registers from the MS561101BA datasheet
  int32_t  range[SRF08_MAX_SENSORS];
  int8_t   sensors;              // the number of sensors present
  int8_t   current;              // the current sensor being read
  uint8_t  state;
  uint32_t deadline;
} srf08_ctx;


// read uncompensated temperature value: send command first
void Sonar_init() {
  memset(&srf08_ctx, 0, sizeof(srf08_ctx));
  srf08_ctx.deadline = 4000000;
}

// this function works like readReg accept a failed read is a normal expectation
// use for testing the existence of sensors on the i2c bus
// a 0xffff code is returned if the read failed
uint16_t i2c_try_readReg(uint8_t add, uint8_t reg) {
  uint16_t count = 255;
  i2c_rep_start(add<<1);  // I2C write direction
  i2c_write(reg);        // register selection
  i2c_rep_start((add<<1)|1);  // I2C read direction
  
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT))) {
    count--;
    if (count==0) {              //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      return 0xffff;  // return failure to read
    }
  }
  
  uint8_t r = TWDR;
  i2c_stop();
  return r;  
}

// read a 16bit unsigned int from the i2c bus
uint16_t i2c_readReg16(int8_t addr, int8_t reg) {
  uint8_t b[2];
  i2c_read_reg_to_buf(addr, reg, &b, sizeof(b));
  return (b[0]<<8) | b[1];
}

void i2c_srf08_change_addr(int8_t current, int8_t moveto) {
  // to change a srf08 address, we must write the following sequence to the command register
  // this sequence must occur as 4 seperate i2c transactions!!
  //   A0 AA A5 [addr]
  i2c_writeReg(current, SRF08_REV_COMMAND, 0xA0);  delay(30);
  i2c_writeReg(current, SRF08_REV_COMMAND, 0xAA);  delay(30);
  i2c_writeReg(current, SRF08_REV_COMMAND, 0xA5);  delay(30);
  i2c_writeReg(current, SRF08_REV_COMMAND, moveto);  delay(30); // now change i2c address
  blinkLED(5,1,2);
  #if defined(BUZZER)
   alarmArray[7] = 2;
  #endif
}

// discover previously known sensors and any new sensor (move new sensors to assigned area)
void i2c_srf08_discover() {
  uint8_t addr;
  uint16_t x;

  // determine how many sensors are plugged in
  srf08_ctx.sensors=0;
  addr = SRF08_SENSOR_FIRST;
  for(int i=0; i<SRF08_MAX_SENSORS && x!=0xff; i++) {
    // read the revision as a way to check if sensor exists at this location
    x = i2c_try_readReg(addr, SRF08_REV_COMMAND);
    if(x!=0xffff) {
      // detected a sensor at this address
      srf08_ctx.sensors++;
      addr += 2;
    }
  }
  
  // do not add sensors if we are already maxed
  if(srf08_ctx.sensors < SRF08_MAX_SENSORS) {
    // now determine if any sensor is on the 'new sensor' address (srf08 default address)
    // we try to read the revision number
    x = i2c_try_readReg(SRF08_DEFAULT_ADDRESS, SRF08_REV_COMMAND);
    if(x!=0xffff) {
      // new sensor detected at SRF08 default address
      i2c_srf08_change_addr(SRF08_DEFAULT_ADDRESS, addr);  // move sensor to the next address
      srf08_ctx.sensors++;
    }
  }
}

void Sonar_update() {
  if (currentTime < srf08_ctx.deadline || (srf08_ctx.state==0 && f.ARMED)) return; 
  srf08_ctx.deadline = currentTime;
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz, SRF08 is ok with this speed
  switch (srf08_ctx.state) {
    case 0: 
      i2c_srf08_discover();
      if(srf08_ctx.sensors>0)
        srf08_ctx.state++; 
      else
        srf08_ctx.deadline += 5000000; // wait 5 secs before trying search again
      break;
    case 1: 
      srf08_ctx.current=0;
      srf08_ctx.state++;
      srf08_ctx.deadline += SRF08_RANGE_SLEEP;
      break;
#if defined(SONAR_MULTICAST_PING)
    case 2:
      // send a ping via the general broadcast address
      i2c_writeReg(0, SRF08_REV_COMMAND, 0x51);  // start ranging, result in centimeters
      srf08_ctx.state++;
      srf08_ctx.deadline += SRF08_RANGE_WAIT;
      break;
    case 3: 
      srf08_ctx.range[srf08_ctx.current] = i2c_readReg16( SRF08_SENSOR_FIRST+(srf08_ctx.current<<1), SRF08_ECHO_RANGE);
      srf08_ctx.current++;
      if(srf08_ctx.current >= srf08_ctx.sensors)
        srf08_ctx.state=1;
      break;
#else
    case 2:
      // send a ping to the current sensor
      i2c_writeReg(SRF08_SENSOR_FIRST+(srf08_ctx.current<<1), SRF08_REV_COMMAND, 0x51);  // start ranging, result in centimeters
      srf08_ctx.state++;
      srf08_ctx.deadline += SRF08_RANGE_WAIT;
      break;
    case 3: 
      srf08_ctx.range[srf08_ctx.current] = i2c_readReg16(SRF08_SENSOR_FIRST+(srf08_ctx.current<<1), SRF08_ECHO_RANGE);
      srf08_ctx.current++;
      if(srf08_ctx.current >= srf08_ctx.sensors)
        srf08_ctx.state=1;
      else
        srf08_ctx.state=2; 
      break;
#endif
  } 
sonarAlt = srf08_ctx.range[0]; //tmp
}
#else
inline void Sonar_init() {}
void Sonar_update() {}
#endif



void initSensors() {
  delay(200);
  POWERPIN_ON;
  delay(100);
  i2c_init();
  delay(100);
  if (GYRO) Gyro_init();
  if (BARO) Baro_init();
  if (MAG) Mag_init();
  if (ACC) ACC_init();
  if (SONAR) Sonar_init();
  f.I2C_INIT_DONE = 1;
}
