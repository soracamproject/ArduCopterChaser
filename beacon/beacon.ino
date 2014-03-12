#include <BC_Compat.h>
#include <FastSerial.h>
#include <Bounce2.h>
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"
#include "../../ArduCopter/chaser_defines.h"

// ***********************************************************************************
// シリアルポート
// ***********************************************************************************
FastSerialPort1(gps_serial);	// GPS用
FastSerialPort2(xbee_serial);	// XBee用
//#define gps_serial  Serial
//#define xbee_serial Serial2

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
static uint8_t state;			// ビーコンステート
static uint8_t substate;		// サブステート

// ステートに入った際に必ず実行される部分のマクロ
#define S_INIT       substate=0;prev_ms=now_ms;first_time=false
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
	
	state = BEACON_INIT;
	//state = BEACON_DEBUG;		// デバッグモード
	
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
	
	button1.update();
	button2.update();
	
	if((now_us - prev_us) > 20000){		// 50Hz狙い
		switch(state){
			// ■■■■■INITステート■■■■■
			case BEACON_INIT:
			if(first_time){
				S_INIT;
				control_led(0,0,0,0);
			}
			
			// ■毎回実行■
			// ボタン1が押されたら次のステートへ
			//if(button2.read() == HIGH){
			//	change_state(BEACON_READY);
			//}
			if(button1.read() == HIGH){
				change_state(BEACON_READY);
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
				control_led(1,0,0,0);
			}
			
			// ■毎回実行■
			// *ToDo*
			// スイッチ２が押されたらスロットル0でBEACON_LANDに移行
			if(button2.read() == HIGH){
				substate = 90;
			}
			
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
				// 3秒待ってスロットルを上げる
				if((now_ms - prev_ss_ms) > 3000){
					send_change_chaser_state_cmd(CHASER_READY);
					SS_INCREMENT;
				}
				break;
				
				case 5:
				// 3秒待って次のステートへ
				if((now_ms - prev_ss_ms) > 3000){
					change_state(BEACON_TAKEOFF);
				}
				break;
				
				// 90番台は緊急終了
				case 90:
				// スロットル０
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
				control_led(0,1,0,0);
			}
			
			// ■毎回実行■
			// beacon位置情報を定期的に送信
			if((now_ms - prev_et_ms) > 200){
				//xbee_serial.println(beacon_loc_data.lat);
				send_beacon_loc(beacon_loc_data.lat,beacon_loc_data.lon,beacon_loc_data.pressure);
				prev_et_ms = now_ms;
			}
			// スイッチ２が押されたらスロットル0でBEACON_LANDに移行
			if(button2.read() == HIGH){
				change_state(BEACON_LAND);
			}
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
				prev_et_ms = now_ms;
			}
			// ボタン1が押されたらCHASE開始
			if(button1.read() == HIGH){
				change_state(BEACON_CHASE);
			}
			// **TODO**
			// フェールセーフ
			
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
				//uint32_t tmp_t_ms  = millis();	// 速度確認デバッグ用
				send_beacon_loc(beacon_loc_data.lat,beacon_loc_data.lon,beacon_loc_data.pressure);
				//uint16_t tmp_dt_ms = millis();	// 速度確認デバッグ用
				//xbee_serial.println(tmp_t_ms);	// 速度確認デバッグ用
				//xbee_serial.println(tmp_dt_ms);	// 速度確認デバッグ用
				prev_et_ms = now_ms;
			}
			if(button2.read() == HIGH){
				change_state(BEACON_LAND);
			}
			// **ToDo**
			// スイッチ１が押されたらSTAYに戻る(CHASER_LED消灯)
			
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
			
			case BEACON_DEBUG:
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


