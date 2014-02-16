// CHASERステート変更コマンド送信
// uint8_t state			chaser_state（chaser_defines.h参照）
static void send_change_chaser_state_cmd(uint8_t state){
	send_chaser_cmd(1,state,0);
}

// CHASER用スロットル操作コマンド送信
// uint16_t throttle		スロットル値（0-1000, control_modeがCHASER時のみ作用する）
static void send_change_throttle_cmd_for_chaser(uint16_t throttle){
	send_chaser_cmd(2,0,throttle);
}

// CHASER用アーム操作コマンド送信
static void send_arm_cmd_for_chaser(){
	send_chaser_cmd(3,0,0);
}

// CHASER操作コマンド送信
// uint8_t command			0: 何もしない, 1: CHASERステート変更, 2: スロットル値変更, 3: アーム命令
// uint8_t state			chaser_state（chaser_defines.h参照）
// uint16_t throttle		スロットル値（0-1000, control_modeがCHASER時のみ作用する）
static void send_chaser_cmd(uint8_t command, uint8_t state, uint16_t throttle){
	uint8_t system_id = 20;			// 実績値20
	uint8_t component_id = 200;		// 実績値200
	
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;
	
	mavlink_msg_chaser_cmd_pack(system_id, component_id, &msg, command, state, throttle);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	xbee_serial.write(buf, len);
}

// CHASER用ビーコン位置情報送信
static void send_beacon_loc(int32_t lat, int32_t lon, int16_t alt){
	uint8_t system_id = 20;			// 実績値20
	uint8_t component_id = 200;		// 実績値200
	
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;
	
	mavlink_msg_chaser_beacon_location_pack(system_id, component_id, &msg, lat, lon, alt);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	xbee_serial.write(buf, len);
}






