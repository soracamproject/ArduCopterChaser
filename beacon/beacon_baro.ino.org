// ******************************************************
// ***** ビーコン用気圧センサ(baro)操作用プログラム *****
// ******************************************************
// module: I2C Barometer MS561101BA
// specs are here: http://www.meas-spec.com/downloads/MS5611-01BA03.pdf
// useful info on pages 7 -> 12


// ***********************************************************************************
// baro関連define
// ***********************************************************************************
// アドレス
#define MS561101BA_ADDRESS 0x77 //CBR=0 0xEE I2C address when pin CSB is connected to LOW (GND)
//#define MS561101BA_ADDRESS 0x76 //CBR=1 0xEC I2C address when pin CSB is connected to HIGH (VCC)

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


// ***********************************************************************************
// baro関連変数
// ***********************************************************************************
static struct {
  // sensor registers from the MS561101BA datasheet
  uint16_t c[7];
  union {uint32_t val; uint8_t raw[4]; } ut; //uncompensated T
  union {uint32_t val; uint8_t raw[4]; } up; //uncompensated P
  uint8_t  state;
  uint16_t deadline;
} ms561101ba_ctx;


// ***********************************************************************************
// baro関連関数
// ***********************************************************************************
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

void baro_init(){
	delay(10);
	i2c_MS561101BA_reset();
	delay(100);
	i2c_MS561101BA_readCalibration();
	delay(10);
	i2c_MS561101BA_UT_Start(); 
	ms561101ba_ctx.deadline = now_us+10000; 
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

// use float approximation instead of int64_t intermediate values
// does not use 2nd order compensation under -15 deg
void i2c_MS561101BA_Calculate() {
	int32_t delt;
	
	float dT = (int32_t)ms561101ba_ctx.ut.val - (int32_t)((uint32_t)ms561101ba_ctx.c[5] << 8);
	float off = ((uint32_t)ms561101ba_ctx.c[2] <<16) + ((dT * ms561101ba_ctx.c[4]) /((uint32_t)1<<7));
	float sens = ((uint32_t)ms561101ba_ctx.c[1] <<15) + ((dT * ms561101ba_ctx.c[3]) /((uint32_t)1<<8));
	baro_temp = (dT * ms561101ba_ctx.c[6])/((uint32_t)1<<23);

	if(baro_temp < 0) { // temperature lower than 20st.C 
		delt = baro_temp;
		delt = 5*delt*delt;
		off -= delt>>1; 
		sens -= delt>>2;
	}
	
	baro_temp += 2000;
	baro_pressure_raw = (( (ms561101ba_ctx.up.val * sens ) /((uint32_t)1<<21)) - off)/((uint32_t)1<<15);
}

//return 0: no data available, no computation ;  1: new value available  ; 2: no new value, but computation time
uint8_t baro_update() {                          // first UT conversion is started in init procedure
	if (ms561101ba_ctx.state == 2) {               // a third state is introduced here to isolate calculate() and smooth timecycle spike
		ms561101ba_ctx.state = 0;
		i2c_MS561101BA_Calculate();
	return 2;
	}
	if ((int16_t)(now_us - ms561101ba_ctx.deadline)<0) return 0;
	ms561101ba_ctx.deadline = now_us+10000;  // UT and UP conversion take 8.5ms so we do next reading after 10ms 
	if (ms561101ba_ctx.state == 0) {
		i2c_MS561101BA_UT_Read();
		i2c_MS561101BA_UP_Start();
		baro_common();                              // moved here for less timecycle spike
		ms561101ba_ctx.state = 1;
		return 1;
	} else {
		i2c_MS561101BA_UP_Read();
		i2c_MS561101BA_UT_Start();
		ms561101ba_ctx.state = 2;
		return 2;
	}
}

void baro_common() {
	static int32_t baro_hist[16];
	static uint8_t baro_hist_index = 0;
	int32_t baro_pressure_sum = 0;
	
	uint8_t baro_hist_index_next = (baro_hist_index + 1);
	if (baro_hist_index_next == 16){
		baro_hist_index_next = 0;
	}
	baro_hist[baro_hist_index] = baro_pressure_raw;
	
	for(uint8_t i=0; i<16; i++){
		baro_pressure_sum += baro_hist[baro_hist_index];
	}
	baro_pressure = baro_pressure_sum>>4;	// 4ビットシフト演算（＝16分の1）
	
	baro_hist_index = baro_hist_index_next;
}