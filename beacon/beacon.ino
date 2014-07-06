#include <BC_Compat.h>
#include <FastSerial.h>
#include <Bounce2.h>
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"
#include "../../ArduCopter/chaser_defines.h"

// ***********************************************************************************
// シリアルポート
// ***********************************************************************************
// ビーコンプロト3.0用
FastSerialPort1(gps_serial);	// GPS用
FastSerialPort2(xbee_serial);	// XBee用
// ビーコンプロト3.1用
//FastSerialPort2(gps_serial);	// GPS用
//FastSerialPort3(xbee_serial);	// XBee用

// ***********************************************************************************
// メイン変数
// ***********************************************************************************
static uint32_t now_us = 0;		// 今回時刻格納変数[us]
static uint32_t now_ms = 0;		// 今回時刻格納変数[ms]
static uint32_t prev_us = 0;	// 前回時刻格納変数[us]
static uint32_t prev_ms = 0;	// 前回時刻格納変数[ms]
static uint32_t prev_et_ms = 0;	// 前回時刻格納変数（毎回実行部 every time）[ms]
static uint32_t prev_ss_ms = 0;	// 前回時刻格納変数（サブステートコントロール部 sub state）[ms]
static bool first_time = true;	// ステートに入るのが最初かどうか
static bool blink_on = false;	// LED点滅フラグ
static uint8_t state;			// ビーコンステート
static uint8_t substate;		// サブステート

// ステートに入った際に必ず実行される部分のマクロ
#define S_INIT       substate=0;prev_ms=now_ms;first_time=false;blink_on=true
// サブステートをひとつ進めるマクロ
#define SS_INCREMENT substate++;prev_ss_ms=now_ms


// ***********************************************************************************
// GPS関連変数
// ***********************************************************************************
static struct {
	int32_t lat;
	int32_t lon;
	int32_t pressure;
} beacon_loc_data;

// ***********************************************************************************
// 気圧センサ(baro)関連変数
// ***********************************************************************************
// 圧力の単位は[x100 mbar][x1000kPa] ※101300、つまり[Pa]
// 温度の単位は[x100 deg.C] ※2300が23deg.C
int32_t baro_pressure_raw;	// センサ生値（取得中に使用）
int32_t baro_pressure_sum;	// 平均を取るための積算値（16個積算して4ビットシフト）
int32_t baro_pressure;		// センサ平均値
int16_t baro_temp;			// センサ温度（何も手を入れていない）

// ***********************************************************************************
// I2C関連変数
// ***********************************************************************************
int16_t i2c_errors_count = 0;

// ***********************************************************************************
// LED関連変数および宣言
// ***********************************************************************************
#define LED1   2
#define LED2   5
#define LED3   7
#define LED4   9
#define BLINK_INTVL_MS  700	//LED点滅間隔[ms]

// ***********************************************************************************
// ボタン関連変数および宣言
// ***********************************************************************************
#define BUTTON1   33
#define BUTTON2   32
Bounce button1 = Bounce();
Bounce button2 = Bounce();


// ***********************************************************************************
// 主要部分
// ***********************************************************************************
void setup()
{
	// LED初期化と全点灯
	pinMode(LED1, OUTPUT);	// R
	pinMode(LED2, OUTPUT);	// Y
	pinMode(LED3, OUTPUT);	// G
	pinMode(LED4, OUTPUT);	// B
	control_led(1,1,1,1);
	
	// GPS初期化
	init_gps();
	
	// XBee初期化
	xbee_serial.begin(57600);
	
	// Baro初期化
	baro_init();
	
	// BUTTON初期化
	pinMode(BUTTON1, INPUT);
	pinMode(BUTTON2, INPUT);
	button1.attach(BUTTON1);
	button2.attach(BUTTON2);
	button1.interval(50);			//たぶんチャタ防止間隔5ms
	button2.interval(50);			//たぶんチャタ防止間隔5ms
	
	// LED全消灯
	control_led(0,0,0,0);
	
	// とりあえず2秒待ち
	//delay(2000);
	
	// 初期ステート設定
	state = BEACON_INIT;
	
	// 前回時間の初期化
	prev_us = micros();
	prev_ms = millis();
}

// *****開発の考え方（暫定）*****
// 50Hzで駆動されるメイン制御部とそれ以外の時間に実行されるセンサ取得部
// メイン制御部はステートコントロールをしている
// 各ステート内は

void loop(){
	now_us = micros();
	now_ms = millis();
	
	if((now_us - prev_us) > 20000){		// 50Hz狙い
		switch(state){
			// ■■■■■INITステート■■■■■
			case BEACON_INIT:
			if(first_time){
				S_INIT;
				control_led(1,0,0,0);
			}
			
			// ■毎回実行■
			// ボタン1が押されたら次のステートへ
			if(button1.update()==1 && button1.read() == HIGH){
				change_state(BEACON_READY);
			}
			// ボタン2が押されたらデバッグモードへ
			if(button2.update()==1 && button2.read() == HIGH){
				//change_state(BEACON_LAND);
				change_state(BEACON_DEBUG);
			}
			
			// ■サブステート実行■
			switch(substate){
				case 0:
				// 何もしない
				break;
			}
			break;
			
			
			
			// ■■■■■READYステート■■■■■
			case BEACON_READY:
			if(first_time){
				S_INIT;
			}
			
			// ■毎回実行■
			// CHASER_READYの時のみ、beacon位置情報を定期的に送信
			if(substate == 5 && (now_ms - prev_et_ms) > 200){	// substateで定義しているため番号変化に注意
				send_beacon_loc(beacon_loc_data.lat,beacon_loc_data.lon,beacon_loc_data.pressure);
				prev_et_ms = now_ms;
			}
			// スイッチ２が押されたらスロットル0でBEACON_LANDに移行
			if(button2.update()==1 && button2.read() == HIGH){
				substate = 90;
			}
			if(blink_on){blink_led(0,1,0,0);};
			
			// ■サブステート実行■
			switch(substate){
				case 0:
				// 機体stateをCHASER_INITにする
				send_change_chaser_state_cmd(CHASER_INIT);
				SS_INCREMENT;
				break;
				
				case 1:
				// 5秒待って機体をアームする
				if((now_ms - prev_ss_ms) > 5000){
					send_arm_cmd_for_chaser();
					SS_INCREMENT;
				}
				break;
				
				case 2:
				// 10秒待ってスロットルを上げる
				if((now_ms - prev_ss_ms) > 15000){
					send_change_throttle_cmd_for_chaser(250);
					SS_INCREMENT;
				}
				break;
				
				case 3:
				// 3秒待ってスロットルを上げる
				if((now_ms - prev_ss_ms) > 3000){
					send_change_throttle_cmd_for_chaser(0);
					SS_INCREMENT;
				}
				break;
				
				case 4:
				// 3秒待ってREADYに入れる
				if((now_ms - prev_ss_ms) > 3000){
					send_change_chaser_state_cmd(CHASER_READY);
					
					// LED(黄)点灯
					blink_on = false;
					control_led(0,1,0,0);
					SS_INCREMENT;
				}
				break;
				
				case 5:
				// 3秒待ってTAKEOFFステートへ
				if((now_ms - prev_ss_ms) > 3000){
					change_state(BEACON_TAKEOFF);
				}
				break;
				
				// 90番台は緊急終了
				case 90:
				// スロットル０
				blink_on = false;
				control_led(1,1,1,1);
				send_change_throttle_cmd_for_chaser(0);
				SS_INCREMENT;
				break;
				
				case 91:
				// 2秒待ってランドステートへ
				if((now_ms - prev_ss_ms) > 2000){
					change_state(BEACON_LAND);
				}
				break;
				
			}
			break;
			
			
			
			// ■■■■■TAKEOFFステート■■■■■
			case BEACON_TAKEOFF:
			if(first_time){
				S_INIT;
			}
			
			// ■毎回実行■
			// beacon位置情報を定期的に送信
			if((now_ms - prev_et_ms) > 200){
				send_beacon_loc(beacon_loc_data.lat,beacon_loc_data.lon,beacon_loc_data.pressure);
				prev_et_ms = now_ms;
			}
			// スイッチ２が押されたらBEACON_LANDに移行
			if(button2.update()==1 && button2.read() == HIGH){
				change_state(BEACON_LAND);
			}
			if(blink_on){blink_led(0,0,1,0);};
			
			
			// **TODO**
			// フェールセーフ
			
			// ■サブステート実行■
			switch(substate){
				case 0:
				// テイクオフする
				send_change_chaser_state_cmd(CHASER_TAKEOFF);
				SS_INCREMENT;
				break;
				
				case 1:
				// 20秒待って次のステートへ
				if((now_ms - prev_ss_ms) > 20000){
					change_state(BEACON_STAY);
				}
				break;
			}
			break;
			
			
			
			// ■■■■■STAYステート■■■■■
			case BEACON_STAY:
			if(first_time){
				S_INIT;
				control_led(0,0,1,0);
			}
			
			// ■毎回実行■
			// beacon位置情報を定期的に送信
			if((now_ms - prev_et_ms) > 200){
				send_beacon_loc(beacon_loc_data.lat,beacon_loc_data.lon,beacon_loc_data.pressure);
				//xbee_serial.print(beacon_loc_data.lat);		// ビーコンGPS精度デバッグ用
				//xbee_serial.print(',');						// ビーコンGPS精度デバッグ用
				//xbee_serial.println(beacon_loc_data.lon);		// ビーコンGPS精度デバッグ用
				prev_et_ms = now_ms;
			}
			// ボタン1が押されたらCHASE開始
			if(button1.update()==1 && button1.read() == HIGH){
				change_state(BEACON_CHASE);
			}
			// スイッチ２が押されたらBEACON_LANDに移行
			if(button2.update()==1 && button2.read() == HIGH){
				change_state(BEACON_LAND);
			}
			
			// ■サブステート実行■
			switch(substate){
				case 0:
				// ステイする
				send_change_chaser_state_cmd(CHASER_STAY);
				SS_INCREMENT;
				break;
			}
			break;
			
			
			
			
			
			// ■■■■■CHASEステート■■■■■
			case BEACON_CHASE:
			if(first_time){
				S_INIT;
				control_led(0,0,0,1);
			}
			
			// ■毎回実行■
			// beacon位置情報を定期的に送信
			if((now_ms - prev_et_ms) > 200){
				send_beacon_loc(beacon_loc_data.lat,beacon_loc_data.lon,beacon_loc_data.pressure);
				prev_et_ms = now_ms;
			}
			// スイッチ１が押されたらSTAYに戻る
			if(button1.update()==1 && button1.read() == HIGH){
				//change_state(BEACON_STAY);
				change_state(BEACON_DEBUG);
			}
			// スイッチ２が押されたらLANDする
			if(button2.update()==1 && button2.read() == HIGH){
				change_state(BEACON_LAND);
			}
			
			// ■サブステート実行■
			switch(substate){
				case 0:
				// CHASEする
				send_change_chaser_state_cmd(CHASER_CHASE);
				SS_INCREMENT;
				break;
			}
			break;
			
			
			
			
			// ■■■■■LANDステート■■■■■
			case BEACON_LAND:
			if(first_time){
				S_INIT;
				control_led(1,1,1,1);
			}
			
			// ■毎回実行■
			// **ToDo**
			// スイッチ２が押されたらLANDもう一度とかできるかなぁ
			
			// ■サブステート実行■
			switch(substate){
				case 0:
				// LANDする
				send_change_chaser_state_cmd(CHASER_LAND);
				SS_INCREMENT;
				break;
				
				case 1:
				// 10秒待ってINITステートへ
				if((now_ms - prev_ss_ms) > 10000){
					change_state(BEACON_INIT);
				}
				break;
			}
			break;
			
			// ■■■■■DEBUGステート■■■■■
			case BEACON_DEBUG:
			if(first_time){
				S_INIT;
				control_led(1,0,1,0);
				//send_change_chaser_state_cmd(CHASER_INIT);
			}
			
			// ■毎回実行■
			// ボタン2が押されたらLANDさせる
			if(button2.update()==1 && button2.read() == HIGH){
				change_state(BEACON_LAND);
			}
			if((now_ms - prev_et_ms) > 200){
				static uint16_t count;
				send_beacon_loc(beacon_loc_data.lat,10000000,beacon_loc_data.pressure);
				//send_beacon_loc(beacon_loc_data.lat,beacon_loc_data.lon,beacon_loc_data.pressure);
				//xbee_serial.println(count++);
				//xbee_serial.println(beacon_loc_data.lat);
				//xbee_serial.println(beacon_loc_data.lon);
				prev_et_ms = now_ms;
			}
			
			break;
		}
		prev_us = now_us;
		prev_ms = now_ms;
	} else {
		// **ToDo**
		// センサ値取得部
		baro_update();		// for MS baro: I2C set and get: 220 us  -  presure and temperature computation 160 us
		beacon_loc_data.pressure = baro_pressure;
		get_gps_new_data();  // I2C GPS: 160 us with no new data / 1250us! with new data 
	}
}

// LEDの点灯用関数
// （本当はマクロとか組めばいいのだけど書きやすいようにリッチにやってます）
static void control_led(uint8_t one, uint8_t two, uint8_t three, uint8_t four){
	if (one==0){
		digitalWrite(LED1, LOW);
	} else {
		digitalWrite(LED1, HIGH);
	}
	
	if (two==0){
		digitalWrite(LED2, LOW);
	} else {
		digitalWrite(LED2, HIGH);
	}
	
	if (three==0){
		digitalWrite(LED3, LOW);
	} else {
		digitalWrite(LED3, HIGH);
	}
	
	if (four==0){
		digitalWrite(LED4, LOW);
	} else {
		digitalWrite(LED4, HIGH);
	}
}

// ビーコンステート変更関数
// ※first_timeの更新を忘れないように
static void change_state(uint8_t next_state){
	state = next_state;
	first_time = true;
}

static void blink_led(uint8_t one, uint8_t two, uint8_t three, uint8_t four){
	static uint32_t prev_ms = 0;	// 前回時刻格納変数[us]
	static bool state = false;
	
	uint32_t now_ms = millis();
	
	if ((now_ms - prev_ms) > BLINK_INTVL_MS) {
		if(state){
			control_led(0,0,0,0);
		} else {
			control_led(one,two,three,four);
		}
		state = !state;
		prev_ms = now_ms;
	}
}

