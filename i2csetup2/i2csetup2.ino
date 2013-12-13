#define I2C_ADDRESS 30
#define I2C_ROM_ADDRESS 22
#define I2C_SPEED 100000L

#define rgTWISlEna (1<<TWEA)|(1<<TWEN)  // スレーブ ACK応答|(1<<TWIE)
#define rgClrTWInt rgTWISlEna|(1<<TWINT)     // TWINT割り込み要因フラグのクリア

// ***********************************************************************************
// I2C関連変数
// ***********************************************************************************
uint8_t I2CData;
uint8_t a;
uint8_t data_count;

#define BLINK_INTERVAL  90


// ***********************************************************************************
// GPS関連変数
// ***********************************************************************************
static int32_t  GPS_read[2] = {0,0};
static uint8_t  GPS_numSats = 0;
static int16_t  GPS_altitude = 0;                                // GPS altitude      - unit: meter
static uint8_t  GPS_2dfix = 0;
static uint8_t  GPS_3dfix = 0;
static uint16_t	GPS_ground_speed = 0;             // ground speed from gps m/s*100
static uint16_t GPS_ground_ground_course = 0;	          // GPS ground course
static uint32_t GPS_time = 0;
static uint32_t GPS_debug = 0;

static struct
{
	union
	{
		int32_t coord;
		int8_t data[4];
	}lat;

	union
	{
		int32_t coord;
		int8_t data[4];
	}lon;
	
	union
	{
		int16_t coord;
		int8_t data[2];
	}alt;
}gps_data;


#define LAT  0
#define LON  1

struct flags_struct {
  uint8_t GPS_FIX :1 ;
} f;


// ***********************************************************************************
// LED関連変数
// ***********************************************************************************
static uint32_t lastframe_time = 0;
static uint32_t _statusled_timer = 0;
static int8_t   _statusled_blinks = 0;
static boolean  _statusled_state = 0;


// ***********************************************************************************
// MSP関連変数
// ***********************************************************************************
#define MSP_SET_WP		209



void setup()
{
	i2c_init();
	GPS_SerialInit();
	//	sei(); // 全割り込み許可
	digitalWrite(13, HIGH);		//デバッグ用

}


void loop(){
	while (Serial.available()) {
		if (GPS_UBLOX_newFrame(Serial.read())) {
			// We have a valid GGA frame and we have lat and lon in GPS_read_lat and GPS_read_lon, apply moving average filter
			// this is a little bit tricky since the 1e7/deg precision easily overflow a long, so we apply the filter to the fractions
			// only, and strip the full degrees part. This means that we have to disable the filter if we are very close to a degree line
			
			// Think this line was in the wrong place. The way it used to be the lastframe_time was only updated when we had a (3D fix && we have 5 or more sats).
			// This stops the single led blink from indicating a good packet, and the double led blink from indicating a 2D fix
			lastframe_time = millis();
			
			gps_data.lat.coord = GPS_read[LAT];
			gps_data.lon.coord = GPS_read[LON];
		}
	}
	//GPS_NewData();//無駄なコードも多く含むので改良を推奨（性能に影響はないけど）
	//gps_data.lat.coord = GPS_coord[LAT];//データを送信用の箱に格納
	//gps_data.lon.coord = GPS_coord[LON];
	//gps_data.alt.coord = GPS_altitude;

	//gps_data.lat.coord = 777;
	//gps_data.lon.coord = 135791113;
	//gps_data.alt.coord = 10121;
	I2CSlCom();

}


// ***********************************************************************************************************
// I2C general functions
// ************************************************************************************************************

void i2c_init(void) {
  PORTC |= 1<<4;//ポート設定
  PORTC |= 1<<5;//ポート設定
  TWAR = (I2C_ADDRESS<<1); 
//  TWBR = ((F_CPU / I2C_SPEED) - 16) / 2;       // マスターが決める
  TWCR = rgTWISlEna; // enable twi module, no interrupt
}


void I2CSlCom() {

    if(!(TWCR & (1<<TWINT))) {
        return;                                          // TWINTが未セットのとき終了
    }

    // TWINTがセットされているとき
    switch(TWSR & 0xF8) {                      // 状態コード(下位3ビットをマスク)
        case 0x60:          // CB(W)受信
            // ---- CB(W)受信 ----
            SlaveInit(TWDR);                     // CB受信イベント・ハンドラ
            TWCR = rgClrTWInt; // INT要求フラグ・クリア
            data_count = 0;
            break;

        case 0x80:          // Data受信
            // ---- スレーブ受信 ----
            SlaveReceive(TWDR);         // 受信イベント・ハンドラ
            TWCR = rgClrTWInt; // INT要求フラグ・クリア
            break;

        case 0xA8:          // CB(R)受信
            // ---- CB(R)受信 ----
            SlaveInit(TWDR);                // CB受信イベント・ハンドラ
            // スレーブ送信第1バイト取得
            TWDR = SlaveSend();         // スレーブ送信イベント・ハンドラ
                                        //       送信データ取得

            TWCR = rgClrTWInt; // INT要求フラグ・クリア
            break;

        case 0xB8:          // Data送信(ACK受信)
            // ---- スレーブ送信(ACK受信)(スレーブ送信継続) ----
            // スレーブ送信第2バイト以降取得
            TWDR =  SlaveSend();        // スレーブ送信イベント・ハンドラ
                                                       //       送信データ取得

            TWCR = rgClrTWInt; // INT要求フラグ・クリア
            break;

        case 0xC0:          // Data送信(NOACK受信) 最終データ スレーブ送信終了
        case 0xA0:          // スレーブ受信中のストップ・コンディション
            TWCR = rgClrTWInt; // INT要求フラグ・クリア
  //          Serial.print(TWDR);//デバッグ用。ここまで動作確認した
            break;
    }
    TWCR =  (1<<TWEA) | (1<<TWEN) | (1<<TWINT);       // INT要求フラグ・クリア
}


void SlaveInit(uint8_t dat)
{
;
}


void SlaveReceive(uint8_t dat)
{
    I2CData = dat;
 //   Serial.print(I2CData);//デバッグ用
}

/*uint8_t SlaveSend(void) {
//   return (I2CData + 1);     // 過去にマスタから送られてきたデータを送り返す

      a=a+1;
      if(a>10){
        a=0;
      }
	return (a);     // 固定値を送る
}*/
uint8_t SlaveSend(void) {
	uint8_t x;
	switch(data_count)
	{
	case 0:
		x =gps_data.lat.data[0];
		break;
	case 1:
		x =gps_data.lat.data[1];
		break;
	case 2:
		x =gps_data.lat.data[2];
		break;
	case 3:
		x =gps_data.lat.data[3];
		break;
	case 4:
		x =gps_data.lon.data[0];
		break;
	case 5:
		x =gps_data.lon.data[1];
		break;
	case 6:
		x =gps_data.lon.data[2];
		break;
	case 7:
		x =gps_data.lon.data[3];
		break;
	case 8:
		x =gps_data.alt.data[0];
		break;
	case 9:
		x =gps_data.alt.data[1];
		break;
	}
	
	data_count++;
	return(x);
}

