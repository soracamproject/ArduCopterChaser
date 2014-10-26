#include <BC_Common.h>
#include <BC_Math.h>
#include <BC_Bounce.h>
#include <FastSerial.h>
#include <BC_GPS.h>
#include <BC_I2C.h>
#include <BC_InertialSensor.h>
#include <BC_Compass.h>
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"
#include "../../ArduCopter/chaser_defines.h"

#define BEACON_IBIS
//#define BEACON_PHEASANT

// ***********************************************************************************
// シリアルポート
// ***********************************************************************************
#if defined(BEACON_IBIS)
	// ibis用
	FastSerialPort0(console);		// console(デバッグ用)、USB通信用
	FastSerialPort1(gps_serial);	// GPS用
	FastSerialPort2(xbee_serial);	// XBee用
#endif

static bool dummy0;

#if defined(BEACON_PHEASANT)
	// pheasant用
	FastSerialPort0(console);		// console(デバッグ用)、USB通信用
	FastSerialPort2(xbee_serial);	// XBee用
	FastSerialPort3(gps_serial);	// GPS用
#endif

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

// MultiWii移植部、暫定
uint16_t calibratingB = 0;  // baro calibration = get new ground pressure value

// 通信デバッグ用
#define DEBUG_NUM 10
static uint8_t debug_count;
static bool debug_send_flag;
static uint16_t beacon_id[DEBUG_NUM];
static uint16_t copter_id[DEBUG_NUM];
static uint32_t beacon_time_sended[DEBUG_NUM];
static uint32_t copter_time_received[DEBUG_NUM];
static uint32_t beacon_time_received[DEBUG_NUM];


// ステートに入った際に必ず実行される部分のマクロ
#define S_INIT       substate=0;prev_ms=now_ms;first_time=false;blink_on=true
// サブステートをひとつ進めるマクロ
#define SS_INCREMENT substate++;prev_ss_ms=now_ms



// ***********************************************************************************
// 位置関連変数
// ***********************************************************************************
// ビーコン姿勢
static int16_t beacon_roll;		// ロール角、角度x10、0.1deg=1
static int16_t beacon_pitch;	// ピッチ角、角度x10、0.1deg=1
static int16_t beacon_heading;	// 方角、角度、北が0

// GPS位置
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
// LED関連変数および宣言
// ***********************************************************************************
#if defined(BEACON_IBIS)
	// ibis用
	#define LED1   2
	#define LED2   5
	#define LED3   7
	#define LED4   9
#endif

static bool dummy1;

#if defined(BEACON_PHEASANT)
	// pheasant用
	#define LED1   2
	#define LED2   3
	#define LED3   5
	#define LED4   6

#endif

#define BLINK_INTVL_MS  700	//LED点滅間隔[ms]

// ***********************************************************************************
// ボタン関連変数および宣言
// ***********************************************************************************
#if defined(BEACON_IBIS)
	// ibis用
	#define BUTTON1   33
	#define BUTTON2   32
	Bounce button1 = Bounce();
	Bounce button2 = Bounce();
#endif

static bool dummy2;

#if defined(BEACON_PHEASANT)
	// pheasant用
	#define BUTTON1   36
	#define BUTTON2   37
	Bounce button1 = Bounce();
	Bounce button2 = Bounce();
#endif


// ***********************************************************************************
// GPSインスタンス
// ***********************************************************************************
static BC_GPS gps(gps_serial);
static BC_I2C i2c;
static BC_InertialSensor ins(i2c);
static BC_Compass compass(i2c);


// ***********************************************************************************
// 主要部分
// ***********************************************************************************
void setup(){
	// LED初期化と全点灯
	pinMode(LED1, OUTPUT);	// R
	pinMode(LED2, OUTPUT);	// Y
	pinMode(LED3, OUTPUT);	// G
	pinMode(LED4, OUTPUT);	// B
	control_led(1,1,1,1);
	
	// GPS初期化
	gps.init_gps();
	
	// XBee初期化
	xbee_serial.begin(57600);
	
	// console(デバッグ用)初期化
	console.begin(115200);
	
	// 各種センサ類初期化
	i2c.init();		// I2C通信
	delay(100);
	ins.init();			// MPU6050,GYRO+ACC
	compass.init();		// HMC5883C,MAG		//たぶん大丈夫だと思うけどinsより後に初期化で実績有
	
	// BUTTON初期化
	pinMode(BUTTON1, INPUT);
	pinMode(BUTTON2, INPUT);
	button1.attach(BUTTON1);
	button2.attach(BUTTON2);
	button1.interval(50);			//たぶんチャタ防止間隔5ms
	button2.interval(50);			//たぶんチャタ防止間隔5ms
	
	// LED全消灯
	control_led(-1,-1,-1,-1);
	
	// 初期ステート設定
	change_state(BEACON_INIT);
	
	// 前回時間の初期化
	prev_us = micros();
	prev_ms = millis();
}

// loop関数の考え方（暫定版）
// 50Hzで駆動されるメイン制御部とそれ以外の時間に実行されるセンサ取得部で構成される
void loop(){
	// 時刻取得
	now_us = micros();
	now_ms = millis();
	
	if((now_us - prev_us) > 20000){		// 50Hz狙い
		// メイン制御実行
		beacon_main_run();
		
		// 時刻更新
		prev_us = now_us;
		prev_ms = now_ms;
	} else {
		// サブ制御実行
		beacon_sub_run();
	}
}

// メイン制御関数
// 各ステートでスイッチされる
static void beacon_main_run(){
	switch(state){
		case BEACON_INIT:
		beacon_init_run();
		break;
		
		case BEACON_READY:
		beacon_ready_run();
		break;
		
		case BEACON_TAKEOFF:
		beacon_takeoff_run();
		break;
		
		case BEACON_STAY:
		beacon_stay_run();
		break;
		
		case BEACON_CHASE:
		beacon_chase_run();
		break;
		
		case BEACON_CIRCLE:
		beacon_circle_run();
		break;
		
		case BEACON_LAND:
		beacon_land_run();
		break;
		
		case BEACON_DEBUG:
		beacon_debug_run();
		
		default:
		break;
	}
}

// サブ制御関数
// 順に実行される
static void beacon_sub_run(){
	// サブタスクステート
	static uint8_t subtask_state = 0;
	
	switch(subtask_state){
		case 0:
		// 気圧センサ更新
		//baro_update();		// for MS baro: I2C set and get: 220 us  -  presure and temperature computation 160 us
		//beacon_loc_data.pressure = baro_pressure;
		break;
		
		case 1:
		// GPS取得
		gps.get_gps_new_data();  // I2C GPS: 160 us with no new data / 1250us! with new data
		beacon_loc_data.lat = gps.lat_data;
		beacon_loc_data.lon = gps.lon_data;
		break;
		
		case 2:
		// ジャイロセンサ、加速度センサ取得
		ins.get_data();
		break;
		
		case 3:
		// 磁気センサ取得
		compass.read();
		break;
		
		case 4:
		// Mavlink メッセージ受信
		check_input_msg();
		break;
	}
	
	// サブタスクステートを進める
	// 注意：サブタスクを増やしたらこちらの数字も増やすこと
	subtask_state++;
	if(subtask_state > 4){
		subtask_state = 0;
	}
}

// ビーコンステート変更関数
static bool change_state(uint8_t next_state){
	// 現在のステートと同じであれば変更無し
	// (フェールセーフ、実際は使用無し)
	if(state > BEACON_INIT && state == next_state){
		return false;
	}
	
	// ステート開始時の関数を呼ぶ
	switch(next_state){
		case BEACON_INIT:
		beacon_init_start();
		break;
		
		case BEACON_READY:
		beacon_ready_start();
		break;
		
		case BEACON_TAKEOFF:
		beacon_takeoff_start();
		break;
		
		case BEACON_STAY:
		beacon_stay_start();
		break;
		
		case BEACON_CHASE:
		beacon_chase_start();
		break;
		
		case BEACON_CIRCLE:
		beacon_circle_start();
		break;
		
		case BEACON_LAND:
		beacon_land_start();
		break;
		
		case BEACON_DEBUG:
		beacon_debug_start();
		break;
		
		default:
			return false;
		break;
	}
	
	// ステートを変更しtrueを返す
	state = next_state;	
	return true;
}

static void beacon_init_start(){
	substate = 0;
	prev_ms = now_ms;
	blink_on = true;
	control_led(1,-1,-1,-1);
}

static void beacon_ready_start(){
	substate = 0;
	prev_ms = now_ms;
	blink_on = true;
	control_led(-1,-1,-1,-1);
}

static void beacon_takeoff_start(){
	substate = 0;
	prev_ms = now_ms;
	blink_on = true;
	control_led(-1,-1,-1,-1);
}

static void beacon_stay_start(){
	substate = 0;
	prev_ms = now_ms;
	blink_on = true;
	control_led(-1,-1,1,-1);
}

static void beacon_chase_start(){
	substate = 0;
	prev_ms = now_ms;
	blink_on = true;
	control_led(-1,-1,-1,1);
}

static void beacon_circle_start(){
	substate = 0;
	prev_ms = now_ms;
	blink_on = true;
	control_led(-1,1,-1,1);
}

static void beacon_land_start(){
	substate = 0;
	prev_ms = now_ms;
	blink_on = true;
	control_led(1,1,1,1);
}

static void beacon_debug_start(){
	substate = 0;
	prev_ms = now_ms;
	blink_on = true;
	control_led(1,-1,1,-1);
	
	// debug_countを0にする
	debug_count = 0;
	debug_send_flag = true;
	
	// GYRO,ACCのキャリブレーションを開始
	ins.gyro_calib_start();
	ins.acc_calib_start();
	compass.calib_start();
}


static void beacon_init_run(){
	// ボタン1が押されたら次のステートへ
	if(button1.push_check()){
		if(change_state(BEACON_READY)){return;}
	}
	
	// ボタン2が押されたらLANDモードへ
	if(button2.push_check()){
		//change_state(BEACON_LAND);
		if(change_state(BEACON_DEBUG)){return;}
	}
}

static void beacon_ready_run(){
	// スイッチ１が押されたらBEACON_TAKEOFFに移行
	if(button1.push_check()){
		if(change_state(BEACON_TAKEOFF)){return;}
	}
	
	// スイッチ２が押されたらBEACON_LANDに移行
	if(button2.push_check()){
		if(change_state(BEACON_LAND)){return;}
	}
	
	// LED点滅
	if(blink_on){blink_led(0,1,0,0);};
	
	// サブステート実行
	switch(substate){
		case 0:
		// 機体stateをCHASER_INITにする
		send_change_chaser_state_cmd(CHASER_INIT);
		SS_INCREMENT;
		break;
		
		case 1:
		// 10秒待って機体stateをCHASER_READYにする
		if((now_ms - prev_ss_ms) > 10000){
			send_change_chaser_state_cmd(CHASER_READY);
			SS_INCREMENT;
		}
		break;
		
		case 2:
		// 10秒待って機体をアームする
		if((now_ms - prev_ss_ms) > 10000){
			send_arm_cmd_for_chaser();
			
			// LED(黄)点灯
			blink_on = false;
			control_led(-1,1,-1,-1);
			SS_INCREMENT;
		}
		break;
	}
}

static void beacon_takeoff_run(){
	// beacon位置情報を定期的に送信
	if((now_ms - prev_et_ms) > 200){
		send_beacon_loc(beacon_loc_data.lat,beacon_loc_data.lon,beacon_loc_data.pressure);
		prev_et_ms = now_ms;
	}
	
	// スイッチ１が押されたらBEACOM_STAYに移行
	if(substate ==1 && button1.push_check()){
		if(change_state(BEACON_STAY)){return;}
	}
	
	// スイッチ２が押されたらBEACON_LANDに移行
	if(button2.push_check()){
		if(change_state(BEACON_LAND)){return;}
	}
	
	// LED点滅
	if(blink_on){blink_led(0,0,1,0);};
	
	
	// サブステート実行
	switch(substate){
		case 0:
		// テイクオフする
		send_change_chaser_state_cmd(CHASER_TAKEOFF);
		SS_INCREMENT;
		break;
	}
}

static void beacon_stay_run(){
	// beacon位置情報を定期的に送信
	if((now_ms - prev_et_ms) > 200){
		send_beacon_loc(beacon_loc_data.lat,beacon_loc_data.lon,beacon_loc_data.pressure);
		prev_et_ms = now_ms;
	}
	
	// ボタン1が押されたらCHASE開始
	if(button1.push_check()){
		if(change_state(BEACON_CHASE)){return;}
	}
	
	// スイッチ２が押されたらBEACON_LANDに移行
	if(button2.push_check()){
		if(change_state(BEACON_LAND)){return;}
	}
	
	// サブステート実行
	switch(substate){
		case 0:
		// ステイする
		send_change_chaser_state_cmd(CHASER_STAY);
		SS_INCREMENT;
		break;
	}
}

static void beacon_chase_run(){
	// beacon位置情報を定期的に送信
	if((now_ms - prev_et_ms) > 200){
		send_beacon_loc(beacon_loc_data.lat,beacon_loc_data.lon,beacon_loc_data.pressure);
		prev_et_ms = now_ms;
	}
	
	// スイッチ１が押されたらSTAYに戻る
	if(button1.push_check()){
		if(change_state(BEACON_CIRCLE)){return;}
	}
	// スイッチ２が押されたらLANDする
	if(button2.push_check()){
		if(change_state(BEACON_LAND)){return;}
	}
	
	// サブステート実行
	switch(substate){
		case 0:
		// CHASEする
		send_change_chaser_state_cmd(CHASER_CHASE);
		SS_INCREMENT;
		break;
	}
}

static void beacon_circle_run(){
	// beacon位置情報を定期的に送信
	if((now_ms - prev_et_ms) > 200){
		send_beacon_loc(beacon_loc_data.lat,beacon_loc_data.lon,beacon_loc_data.pressure);
		prev_et_ms = now_ms;
	}
	
	// スイッチ１が押されたらSTAYに戻る
	if(button1.push_check()){
		if(change_state(BEACON_STAY)){return;}
	}
	// スイッチ２が押されたらLANDする
	if(button2.push_check()){
		if(change_state(BEACON_LAND)){return;}
	}
	
	// サブステート実行
	switch(substate){
		case 0:
		// CHASEする
		send_change_chaser_state_cmd(CHASER_CIRCLE);
		SS_INCREMENT;
		break;
	}
}


static void beacon_land_run(){
	// サブステート実行
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
}

static void beacon_debug_run(){
	// ボタン2が押されたらLANDさせる
	if(button2.push_check()){
		if(change_state(BEACON_LAND)){return;}
	}
	
	if((now_ms - prev_et_ms) > 1000){
		// 通信速度チェック
		//debug_check_telem();
		
		// GPSチェック
		debug_check_gps();
		
		// センサチェック
		//debug_check_gyro_acc_mag();
		
		prev_et_ms = now_ms;
	}
}


// LEDの点灯用関数
// -1:消灯、0:そのまま、1:点灯
// （本当はマクロとか組めばいいのだけど書きやすいようにリッチにやってます）
static void control_led(int8_t one, int8_t two, int8_t three, int8_t four){
	if (one==-1){
		digitalWrite(LED1, LOW);
	} else if(one==1){
		digitalWrite(LED1, HIGH);
	}
	
	if (two==-1){
		digitalWrite(LED2, LOW);
	} else if(two==1) {
		digitalWrite(LED2, HIGH);
	}
	
	if (three==-1){
		digitalWrite(LED3, LOW);
	} else if(three==1){
		digitalWrite(LED3, HIGH);
	}
	
	if (four==-1){
		digitalWrite(LED4, LOW);
	} else if(four==1){
		digitalWrite(LED4, HIGH);
	}
}

// LEDの点滅用関数
// 1にしたLEDのみ点滅する
static void blink_led(uint8_t one, uint8_t two, uint8_t three, uint8_t four){
	static uint32_t prev_ms = 0;	// 前回時刻格納変数[us]
	static bool state = false;
	
	uint32_t now_ms = millis();
	
	if ((now_ms - prev_ms) > BLINK_INTVL_MS) {
		if(state){
			control_led(-one,-two,-three,-four);
		} else {
			control_led(one,two,three,four);
		}
		state = !state;
		prev_ms = now_ms;
	}
}

static void debug_check_telem(){
	if(debug_count > DEBUG_NUM){return;}
	if(debug_count == DEBUG_NUM){
		for(int8_t i=0; i<DEBUG_NUM; i++){
			console.print("beacon_id=");
			delay(50);
			console.print(beacon_id[i]);
			delay(50);
			console.print(", copter_id=");
			delay(50);
			console.print(copter_id[i]);
			delay(50);
			console.print(", beacon_time_sended=");
			delay(50);
			console.print(beacon_time_sended[i]);
			delay(50);
			console.print(", copter_time_received=");
			delay(50);
			console.print(copter_time_received[i]);
			delay(50);
			console.print(", beacon_time_received=");
			delay(50);
			console.println(beacon_time_received[i]);
			delay(50);
		}
		debug_count++;
	}
	if(debug_count < DEBUG_NUM){
		if(debug_send_flag){
			// 送信フラグがたっていたら、id,timeを記録し機体に送信
			beacon_id[debug_count] = debug_count;
			beacon_time_sended[debug_count] = millis();
			send_debug_cmd_for_chaser(beacon_id[debug_count]);
			debug_send_flag = false;
			console.print("message_sended ");
			console.println(debug_count);
		}
	}
}

static void debug_check_gps(){
	xbee_serial.print("lat=");
	delay(100);
	xbee_serial.print(beacon_loc_data.lat);
	delay(100);
	xbee_serial.print(", lon=");
	delay(100);
	xbee_serial.println(beacon_loc_data.lon);
	delay(100);
}

static void debug_check_gyro_acc_mag(){
	if(ins.calib_ok() && compass.calib_ok()){
		Vector3f accel = ins.get_accel();
		
		xbee_serial.print("gyro:");				delay(20);
		xbee_serial.print(ins.gyroADC[0]);		delay(20);
		xbee_serial.print(", ");				delay(20);
		xbee_serial.print(ins.gyroADC[1]);		delay(20);
		xbee_serial.print(", ");				delay(20);
		xbee_serial.print(ins.gyroADC[2]);		delay(20);
		
		xbee_serial.print("       acc:");		delay(20);
		xbee_serial.print(accel.x);				delay(20);
		xbee_serial.print(", ");				delay(20);
		xbee_serial.print(accel.y);				delay(20);
		xbee_serial.print(", ");				delay(20);
		xbee_serial.print(accel.z);				delay(20);
		
		xbee_serial.print("       mag:");		delay(20);
		xbee_serial.print(compass.magADC[0]);	delay(20);
		xbee_serial.print(", ");				delay(20);
		xbee_serial.print(compass.magADC[1]);	delay(20);
		xbee_serial.print(", ");				delay(20);
		xbee_serial.println(compass.magADC[2]);	delay(20);
	} else if(ins.calib_ok()) {
		xbee_serial.println("gyro+acc calibration ok.");
		delay(20);
	} else if(compass.calib_ok()) {
		xbee_serial.println("mag calibration ok.");
		delay(20);
	} else {
		xbee_serial.println("now calibrating or something wrong.");
		delay(20);
	}
}



