// CHASERステート変更コマンド送信
// uint8_t state			chaser_state（chaser_defines.h参照）
static void send_change_chaser_state_cmd(uint8_t state){
	send_chaser_cmd(1,state,0,0);
}

// CHASER用アーム操作コマンド送信
static void send_arm_cmd_for_chaser(){
	send_chaser_cmd(3,0,0,0);
}

// CHASER用millisコマンド送信（デバッグ用）
static void send_debug_cmd_for_chaser(uint16_t id){
	send_chaser_cmd(4,0,id,0);
}

// CHASER操作コマンド送信
// uint8_t  command		1: CHASERステート変更, 2: 未使用, 3: アーム命令, 4: timeを送信する（デバッグ用）
// uint8_t  p1			0〜255				CHASERステートで使用
// uint16_t p2			0〜65535			現在使用無し
// uint32_t p3			0〜4294967295		time送信に利用（デバッグ用）
static void send_chaser_cmd(uint8_t command, uint8_t p1, uint16_t p2, uint32_t p3){
	uint8_t system_id = 20;			// 実績値20
	uint8_t component_id = 200;		// 実績値200
	
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;
	
	mavlink_msg_chaser_cmd_pack(system_id, component_id, &msg, command, p1, p2, p3);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	xbee_serial.write(buf, len);
}

// CHASER用ビーコン位置情報送信
static void send_beacon_loc(int32_t lat, int32_t lon, int32_t pressure){
	uint8_t system_id = 20;			// 実績値20
	uint8_t component_id = 200;		// 実績値200
	
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;
	
	mavlink_msg_chaser_beacon_location_pack(system_id, component_id, &msg, lat, lon, pressure);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	xbee_serial.write(buf, len);
}

// オフセット再計算指令送信
static void send_recalc_offset(){
	uint8_t system_id = 20;			// 実績値20
	uint8_t component_id = 200;		// 実績値200
	
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;
	
	mavlink_msg_chaser_recalc_offset_pack(system_id, component_id, &msg, 0, 0.f, 0.f);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	xbee_serial.write(buf, len);
}





