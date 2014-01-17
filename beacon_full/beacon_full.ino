#include <BC_Compat.h>
#include <FastSerial.h>
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"
#include "../../ArduCopter/chaser_defines.h"

//XBee用
FastSerialPort0(Serial);

//I2C-GPS用
#define I2C_ADDRESS 30
#define I2C_ROM_ADDRESS 43
#define I2C_SPEED 100000L

uint8_t rawADC[6];

//GPS用
struct GPS_DATA{
	union
	{
		uint32_t coord;
		uint8_t data[4];
	}lat;
	union
	{
		uint32_t coord;
		uint8_t data[4];
	}lon;
	union
	{
		uint16_t coord;
		uint8_t data[2];
	}alt;
};

static GPS_DATA gps_data;

void setup()
{
	// XBee用のシリアルポートを開ける
	Serial.begin(57600);

	// GPS取得用のI2Cの初期化
	i2c_init();
	
	gps_data.lat.coord = 0;
	gps_data.lon.coord = 0;
	gps_data.alt.coord = 0;
	
	// 緊急終了処置
	// 再起動すると実行されるという意味で
	emergency_end_process();
	
	delay(2000);
}


void loop(){
	static uint32_t now = 0;
	static uint8_t state = BEACON_INIT;
	
	switch(state){
		case BEACON_INIT:
			send_change_chaser_state_cmd(CHASER_INIT);
			delay(3000);
			
			state = BEACON_READY;
			//state = BEACON_END;	//デバッグ用
			break;
			
		case BEACON_READY:
			send_arm_cmd(1.0f);		// アーム命令
			delay(10000);
			
			send_change_throttle_cmd_for_chaser(300);
			delay(3000);
			
			send_change_throttle_cmd_for_chaser(0);
			delay(3000);
			
			send_change_chaser_state_cmd(CHASER_READY);
			delay(3000);
			
			state = BEACON_TAKEOFF;
			
			break;
			
		case BEACON_TAKEOFF:
			send_change_chaser_state_cmd(CHASER_TAKEOFF);
			delay(20000);
			
			state = BEACON_LAND;
			state = BEACON_END;		// デバッグ用
			
			break;

		case BEACON_LAND:
			send_change_chaser_state_cmd(CHASER_LAND);
			delay(2000);
			
			state = BEACON_END;
			
			break;
		
		case BEACON_END:
			// 何もしない
			break;
		
		default:
			// 何もしない
			break;
	}
}

static void emergency_end_process(){
	send_change_throttle_cmd_for_chaser(0);
	delay(2000);
	send_change_chaser_state_cmd(CHASER_LAND);
}

static void get_gps_data(){
	//GPSデータ取得部分
	i2c_rep_start(I2C_ADDRESS<<1);//スタートコンディションの発行＋コントロールバイトの送信
	i2c_write(I2C_ROM_ADDRESS);//ROMアドレス送信
	//delay(1500);
	delay(700);
	
	i2c_rep_start((I2C_ADDRESS<<1)|1);//スタートコンディションの再発行+コントロールバイトの送信
	gps_data.lat.data[0] = i2c_readNak();  //ROMデータ受信＋ストップコンディションの発行
	
	i2c_rep_start((I2C_ADDRESS<<1)|1);//スタートコンディションの再発行+コントロールバイトの送信
	gps_data.lat.data[1] = i2c_readNak();  //ROMデータ受信＋ストップコンディションの発行
	
	i2c_rep_start((I2C_ADDRESS<<1)|1);//スタートコンディションの再発行+コントロールバイトの送信
	gps_data.lat.data[2] = i2c_readNak();  //ROMデータ受信＋ストップコンディションの発行
	
	i2c_rep_start((I2C_ADDRESS<<1)|1);//スタートコンディションの再発行+コントロールバイトの送信
	gps_data.lat.data[3] = i2c_readNak();  //ROMデータ受信＋ストップコンディションの発行
	
	i2c_rep_start((I2C_ADDRESS<<1)|1);//スタートコンディションの再発行+コントロールバイトの送信
	gps_data.lon.data[0] = i2c_readNak();  //ROMデータ受信＋ストップコンディションの発行
	
	i2c_rep_start((I2C_ADDRESS<<1)|1);//スタートコンディションの再発行+コントロールバイトの送信
	gps_data.lon.data[1] = i2c_readNak();  //ROMデータ受信＋ストップコンディションの発行
	
	i2c_rep_start((I2C_ADDRESS<<1)|1);//スタートコンディションの再発行+コントロールバイトの送信
	gps_data.lon.data[2] = i2c_readNak();  //ROMデータ受信＋ストップコンディションの発行
	
	i2c_rep_start((I2C_ADDRESS<<1)|1);//スタートコンディションの再発行+コントロールバイトの送信
	gps_data.lon.data[3] = i2c_readNak();  //ROMデータ受信＋ストップコンディションの発行
	
	i2c_rep_start((I2C_ADDRESS<<1)|1);//スタートコンディションの再発行+コントロールバイトの送信
	gps_data.alt.data[0] = i2c_readNak();  //ROMデータ受信＋ストップコンディションの発行
	
	i2c_rep_start((I2C_ADDRESS<<1)|1);//スタートコンディションの再発行+コントロールバイトの送信
	gps_data.alt.data[1] = i2c_readNak();  //ROMデータ受信＋ストップコンディションの発行
}




// ***********************************************************************************************************
// I2C general functions
// ************************************************************************************************************

void i2c_init(void) {
	PORTC |= 1<<4;//ポート設定
	PORTC |= 1<<5;//ポート設定
	TWSR = 0;                                    // no prescaler => prescaler = 1
	TWBR = ((F_CPU / I2C_SPEED) - 16) / 2;       // change the I2C clock rate
	TWCR = 1<<TWEN;                              // enable twi module, no interrupt
}

void i2c_rep_start(uint8_t address) {//マスタがスレーブにデータを要求する
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) ; // send REPEAT START condition
  waitTransmissionI2C();                       // wait until transmission completed（いるか？
  TWDR = address;                              // send device address
  TWCR = (1<<TWINT) | (1<<TWEN);              //TWINTクリア　マスタ送信開始
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
      //neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay
      //i2c_errors_count++;
      break;//この中は、ハングアップした時用の退避コマンドくさい
    }
  }
}


//以下データ処理用　本質的には関係ない
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
