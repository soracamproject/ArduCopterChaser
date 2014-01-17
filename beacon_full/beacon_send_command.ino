// アームコマンド送信 ARM(1),DISARM(0)
static void send_arm_cmd(float param1){
	uint8_t system_id = 20;			// 実績値20
	uint8_t component_id = 200;		// 実績値200
	uint8_t target_system = 1;		// 1が必要（MISSION PLANNERのFull Param ListでSYSで検索）
	uint8_t target_component = MAV_COMP_ID_SYSTEM_CONTROL;
	uint16_t command = MAV_CMD_COMPONENT_ARM_DISARM;
	uint8_t confirmation = 1;		// 実績値1
	float param2 = 0.0f;			// 使用しない
	float param3 = 0.0f;			// 使用しない
	float param4 = 0.0f;			// 使用しない
	float param5 = 0.0f;			// 使用しない
	float param6 = 0.0f;			// 使用しない
	float param7 = 0.0f;			// 使用しない
	
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;
	
	mavlink_msg_command_long_pack(system_id, component_id, &msg, target_system, target_component, command,
								confirmation, param1, param2, param3, param4, param5, param6, param7);
	
	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	Serial.write(buf, len);
}

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
	
	Serial.write(buf, len);
}

static void send_beacon_loc_cmd(){
	
}






