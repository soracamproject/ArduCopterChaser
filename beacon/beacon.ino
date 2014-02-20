#include <BC_Compat.h>
#include <FastSerial.h>
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"
#include "../../ArduCopter/chaser_defines.h"

// ***********************************************************************************
// シリアルポート
// ***********************************************************************************
FastSerialPort1(gps_serial);	// GPS用
FastSerialPort2(xbee_serial);	// XBee用

//#define gps_serial Serial1
//#define xbee_serial Serial2

// ***********************************************************************************
// GPS関連変数
// ***********************************************************************************
static struct {
	int32_t lat;
	int32_t lon;
	int16_t baro;
} beacon_loc_data;



// ***********************************************************************************
// 主要部分
// ***********************************************************************************
void setup()
{
	// GPS初期化
	init_gps();
	beacon_loc_data.lat = 0;
	beacon_loc_data.lon = 0;
	beacon_loc_data.baro = 0;
	
	// XBee初期化
	xbee_serial.begin(57600);
	
	// 緊急終了処置
	// 再起動すると実行されるという意味で
	emergency_end_process();
	
	delay(2000);
}


void loop(){
	static uint32_t t_first_time_ms = 0;
	static bool first_time = true;
	static uint8_t state = BEACON_INIT;
	state = BEACON_DEBUG;		// デバッグモード
	
	switch(state){
		case BEACON_INIT:
			send_change_chaser_state_cmd(CHASER_INIT);
			delay(5000);
			
			state = BEACON_READY;
			
			break;
			
		case BEACON_READY:			
			send_arm_cmd_for_chaser();
			delay(10000);
			
			send_change_throttle_cmd_for_chaser(250);
			delay(3000);
			send_change_throttle_cmd_for_chaser(0);
			delay(3000);
						
			send_change_chaser_state_cmd(CHASER_READY);
			delay(3000);
			
			state = BEACON_TAKEOFF;
			
			break;
			
		case BEACON_TAKEOFF:
			// ステート変更時に実施されるもの
			if (first_time) {
				t_first_time_ms = millis();
				send_change_chaser_state_cmd(CHASER_TAKEOFF);
				first_time = false;
			}
			
			// GPSデータ初期化（ロスト対策）
			beacon_loc_data.lat = 1;
			beacon_loc_data.lon = 1;
			
			// GPSデータ取得
			//get_gps_data();
			
			// ビーコン位置情報送信
			send_beacon_loc(beacon_loc_data.lat,beacon_loc_data.lon,0);
			
			// 一定時間たったら次のステートに移行
			if ((millis()-t_first_time_ms) >= 20000) {
				state = BEACON_STAY;
				//state = BEACON_END;		//デバッグ用
				first_time = true;
			}
			break;
			
		case BEACON_STAY:
			// ステート変更時に実施されるもの
			if (first_time) {
				t_first_time_ms = millis();
				send_change_chaser_state_cmd(CHASER_STAY);
				first_time = false;
			}
			
			// GPSデータ初期化（ロスト対策）
			beacon_loc_data.lat = 1;
			beacon_loc_data.lon = 1;
			
			// ビーコン位置情報送信
			send_beacon_loc(beacon_loc_data.lat,beacon_loc_data.lon,0);
			
			// 一定時間たったら次のステートに移行
			if ((millis()-t_first_time_ms) >= 10000) {
				state = BEACON_CHASE;
				first_time = true;
			}
			break;
			
		case BEACON_CHASE:
			// ステート変更時に実施されるもの
			if (first_time) {
				t_first_time_ms = millis();
				send_change_chaser_state_cmd(CHASER_CHASE);
				first_time = false;
			}
			
			// GPSデータ初期化（ロスト対策）
			beacon_loc_data.lat = 1;
			beacon_loc_data.lat = 1;
			
			// ビーコン位置情報送信
			send_beacon_loc(beacon_loc_data.lat,beacon_loc_data.lon,0);
			
			// 一定時間たったら次のステートに移行
			if ((millis()-t_first_time_ms) >= 600000) {
				state = BEACON_LAND;
				first_time = true;
			}
			break;
			
		case BEACON_LAND:
			send_change_chaser_state_cmd(CHASER_LAND);
			delay(2000);
			
			state = BEACON_END;
			
			break;
		
		case BEACON_DEBUG:
			//state = BEACON_END;
			get_gps_new_data();
			xbee_serial.println(beacon_loc_data.lat);
			xbee_serial.println(beacon_loc_data.lon);
			delay(1000);
			
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



