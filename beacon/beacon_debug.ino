#define BEACON_DEBUG_ACTIVATE

#if defined(BEACON_DEBUG_ACTIVATE)

// 通信デバッグ用
#define DEBUG_NUM 10
static uint8_t debug_count;
static bool debug_send_flag;
static uint16_t beacon_id[DEBUG_NUM];
static uint16_t copter_id[DEBUG_NUM];
static uint32_t beacon_time_sended[DEBUG_NUM];
static uint32_t copter_time_received[DEBUG_NUM];
static uint32_t beacon_time_received[DEBUG_NUM];

static void debug_init(){
	//debug_count = 0;
	//debug_send_flag = true;
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
	xbee_serial.print(millis());					delay(20);
	xbee_serial.print(",");							delay(20);
	xbee_serial.print(beacon_loc_data.lat);			delay(20);
	xbee_serial.print(",");							delay(20);
	xbee_serial.print(beacon_loc_data.lon);			delay(20);
	xbee_serial.print(",");							delay(20);
	xbee_serial.print(gps.num_sat());				delay(20);
	xbee_serial.print(",");							delay(20);
	xbee_serial.println(gps.hdop());				delay(20);
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

static void debug_check_control_mode(){
	/*
	if(flags.control_mode_chaser){
		control_led(0,1,0,-1);
	} else {
		control_led(0,-1,0,1);
	}
	*/
}



#endif	// end of BEACON_DEBUG_ACTIVATE
