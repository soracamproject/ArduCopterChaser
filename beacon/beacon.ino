#include <BC_Compat.h>
#include <FastSerial.h>
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"
#include "../../ArduCopter/chaser_defines.h"

// ***********************************************************************************
// シリアルポート
// ***********************************************************************************
FastSerialPort1(gps_serial);	// GPS用
FastSerialPort2(xbee_serial);	// XBee用


// ***********************************************************************************
// メイン変数
// ***********************************************************************************
static uint32_t now_us = 0;		// 今回時刻格納変数[us]
static uint32_t now_ms = 0;		// 今回時刻格納変数[ms]
static uint32_t prev_us = 0;	// 前回時刻格納変数[us]
static uint32_t prev_ms = 0;	// 前回時刻格納変数[ms]
static bool first_time = true;	// ステートに入るのが最初かどうか
static uint8_t state = BEACON_INIT;		// ビーコンステート

static struct{
	uint8_t substate;
	uint32_t prev_ms;
} sc[BEACON_END+1];		// ステートコントロール用変数

// ***********************************************************************************
// GPS関連変数
// ***********************************************************************************
static struct {
	int32_t lat;
	int32_t lon;
	int16_t baro;
} beacon_loc_data;

// ***********************************************************************************
// 気圧センサ(baro)関連変数
// ***********************************************************************************
int32_t baro_pressure;
int16_t baro_temp;
int32_t baro_pressure_sum;

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
// 主要部分
// ***********************************************************************************
void setup()
{
	// GPS初期化
	init_gps();
	
	// XBee初期化
	xbee_serial.begin(57600);
	
	// LED初期化
	pinMode(LED1, OUTPUT);	// R
	pinMode(LED2, OUTPUT);	// Y
	pinMode(LED3, OUTPUT);	// G
	pinMode(LED4, OUTPUT);	// B
	digitalWrite(LED1, LOW);
	digitalWrite(LED2, LOW);
	digitalWrite(LED3, LOW);
	digitalWrite(LED4, LOW);
	
	// とりあえず10秒待ち
	delay(1000);
	
	state = BEACON_DEBUG;		// デバッグモード
	
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
			case BEACON_INIT:
			// １．ステートに最初に入った際に実行される部分
			if(first_time){
				sc[state].substate = 0;
				sc[state].prev_ms = now_ms;
				first_time = false;
			}
			
			// ２．毎回実行される部分
			// *ToDo*
			// スイッチ押されたらREADYステートへ
			
			// ３．サブステートコントロール部分
			switch(sc[state].substate){
				case 0:
				// 何もしない
				// INITステートではスイッチが押されたら次のステートへ_ms;
				break;
			}
			break;
			
			case BEACON_READY:
			// １．ステートに最初に入った際に実行される部分
			if(first_time){
				sc[state].substate = 0;
				sc[state].prev_ms = now_ms;
				first_time = false;
			}
			
			// ２．毎回実行される部分
			// *ToDo*
			// スイッチ押されたらスロットル0でCHASER_INITに戻す（ディスアーム）
			
			
			// ３．サブステートコントロール部分
			switch(sc[state].substate){
				case 0:
				// 機体stateをCHASER_INITにする
				send_change_chaser_state_cmd(CHASER_INIT);
				sc[state].substate++;
				sc[state].prev_ms = now_ms;
				break;
				
				case 1:
				// 5秒待って機体をアームする
				if((now_ms - sc[state].prev_ms) > 5000){
					send_arm_cmd_for_chaser();
					sc[state].substate++;
					sc[state].prev_ms = now_ms;
				}
				break;
				
				case 2:
				// 10秒待ってスロットルを上げる
				if((now_ms - sc[state].prev_ms) > 10000){
					send_change_throttle_cmd_for_chaser(250);
					sc[state].substate++;
					sc[state].prev_ms = now_ms;
				}
				break;
				
				case 3:
				// 2秒待ってスロットルを上げる
				if((now_ms - sc[state].prev_ms) > 2000){
					send_change_throttle_cmd_for_chaser(0);
					sc[state].substate++;
					sc[state].prev_ms = now_ms;
				}
				break;
				
				case 4:
				// 2秒待ってスロットルを上げる
				if((now_ms - sc[state].prev_ms) > 2000){
					send_change_chaser_state_cmd(CHASER_READY);
					sc[state].substate++;
					sc[state].prev_ms = now_ms;
				}
				break;
				
				case 5:
				// 2秒待って次のステートへ
				if((now_ms - sc[state].prev_ms) > 2000){
					state = BEACON_TAKEOFF;
					first_time = true;
				}
				break;
			}
			break;
			
			case BEACON_TAKEOFF:
			// １．ステートに最初に入った際に実行される部分
			if(first_time){
				sc[state].substate = 0;
				sc[state].prev_ms = now_ms;
				first_time = false;
			}
			
			// ２．毎回実行される部分
			// *ToDo*
			// ①beacon位置情報を定期的に送信
			// ②フェールセーフ（検討中・・・）
			
			// ３．サブステートコントロール部分
			switch(sc[state].substate){
				case 0:
				// テイクオフする
				send_change_chaser_state_cmd(CHASER_TAKEOFF);
				sc[state].substate = 1;
				sc[state].prev_ms = now_ms;
				break;
				
				case 1:
				// 20秒待って次のステートへ
				if((now_ms - sc[state].prev_ms) > 20000){
					state = BEACON_TAKEOFF;
					first_time = true;
				}
				break;
			}
			break;
			
			case BEACON_STAY:
			// １．ステートに最初に入った際に実行される部分
			if(first_time){
				sc[state].substate = 0;
				sc[state].prev_ms = now_ms;
				first_time = false;
				// **ToDo**
				// STAY_LED点灯
			}
			
			// ２．毎回実行される部分
			// *ToDo*
			// ①beacon位置情報を定期的に送信
			// ②スイッチ１が押されたらフェールセーフ（検討中・・・）
			// ③スイッチ２が押されたらCHASEに移行(STAY_LED消灯)
			
			// ３．サブステートコントロール部分
			switch(sc[state].substate){
				case 0:
				// ステイする
				send_change_chaser_state_cmd(CHASER_STAY);
				sc[state].substate = 1;
				sc[state].prev_ms = now_ms;
				break;
				
				case 1:
				// 何もしない
				// BEACON_STAYの場合、移行はスイッチ操作で実施する
				break;
			}
			break;
			
			case BEACON_CHASE:
			// １．ステートに最初に入った際に実行される部分
			if(first_time){
				sc[state].substate = 0;
				sc[state].prev_ms = now_ms;
				first_time = false;
				// **ToDo**
				// CHASER_LED点灯
			}
			
			// ２．毎回実行される部分
			// *ToDo*
			// ①beacon位置情報を定期的に送信
			// ②スイッチ１が押されたらLAND（検討中・・・）
			// ③スイッチ２が押されたらSTAYに戻る(CHASER_LED消灯)
			
			// ３．サブステートコントロール部分
			switch(sc[state].substate){
				case 0:
				// CHASEする
				send_change_chaser_state_cmd(CHASER_CHASE);
				sc[state].substate = 1;
				sc[state].prev_ms = now_ms;
				break;
				
				case 1:
				// 何もしない
				// BEACON_CHASEの場合、移行はスイッチ操作で実施する
				break;
			}
			break;
			
			case BEACON_LAND:
			// １．ステートに最初に入った際に実行される部分
			if(first_time){
				sc[state].substate = 0;
				sc[state].prev_ms = now_ms;
				first_time = false;
				// **ToDo**
				// LAND_LED点灯
			}
			
			// ２．毎回実行される部分
			// *ToDo*
			// スイッチ２が押されたらLANDもう一度とかできるかなぁ
			
			// ３．サブステートコントロール部分
			switch(sc[state].substate){
				case 0:
				// LANDする
				send_change_chaser_state_cmd(CHASER_LAND);
				sc[state].substate = 1;
				sc[state].prev_ms = now_ms;
				break;
				
				case 1:
				// 10秒待って次のステートへ
				if((now_ms - sc[state].prev_ms) > 10000){
					state = BEACON_END;
					first_time = true;
					// **ToDo**
					// LAND_LED消灯
				}
				break;
			}
			break;
			
			case BEACON_DEBUG:
			// １．ステートに最初に入った際に実行される部分
			if(first_time){
				sc[state].substate = 0;
				sc[state].prev_ms = now_ms;
				first_time = false;
			}
			
			// ２．毎回実行される部分
			
			// ３．サブステートコントロール部分
			switch(sc[state].substate){
				case 0:
				// LED1点灯
				digitalWrite(LED1, HIGH);
				sc[state].substate++;
				sc[state].prev_ms = now_ms;
				break;
				
				case 1:
				// 5秒待ってLED2点灯
				if((now_ms - sc[state].prev_ms) > 5000){
					digitalWrite(LED2, HIGH);
					sc[state].substate++;
					sc[state].prev_ms = now_ms;
				}
				break;
				
				case 2:
				// 5秒待ってLED3点灯
				if((now_ms - sc[state].prev_ms) > 5000){
					digitalWrite(LED3, HIGH);
					sc[state].substate++;
					sc[state].prev_ms = now_ms;
				}
				break;
				
				case 3:
				// 5秒待ってLED4点灯
				if((now_ms - sc[state].prev_ms) > 5000){
					digitalWrite(LED4, HIGH);
					sc[state].substate++;
					sc[state].prev_ms = now_ms;
				}
				break;
				
				case 4:
				// 5秒待ってLED全消灯
				if((now_ms - sc[state].prev_ms) > 5000){
					digitalWrite(LED1, LOW);
					digitalWrite(LED2, LOW);
					digitalWrite(LED3, LOW);
					digitalWrite(LED4, LOW);
					sc[state].substate++;
					sc[state].prev_ms = now_ms;
				}
				break;
				
				case 5:
				// 何もしない
				break;
			}
			break;
			
			case BEACON_END:
			// １．ステートに最初に入った際に実行される部分
			if(first_time){
				sc[state].substate = 0;
				sc[state].prev_ms = now_ms;
				first_time = false;
			}
			
			// ２．毎回実行される部分
			// *ToDo*
			// スイッチ１が押されたらINIT
			
			// ３．サブステートコントロール部分
			switch(sc[state].substate){
				case 0:
				// 何もしない
				// ENDステートではスイッチが押されたらINITステートに移動
				break;
			}
			break;
			
			default:
			// 何もしない
			break;
		}
		prev_us = now_us;
		prev_ms = now_ms;
	} else {
		// **ToDo**
		// センサ値取得部
	}
}