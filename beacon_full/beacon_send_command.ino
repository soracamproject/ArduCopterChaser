// アームコマンド送信 ARM(1),DISARM(0)
void send_arm_cmd(float param1){
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

// CHASER操作コマンド送信
// uint8_t state			chaser_state（chaser_defines.h参照）
void send_change_chaser_state_cmd(uint8_t state){
	uint8_t system_id = 20;			// 実績値20
	uint8_t component_id = 200;		// 実績値200
	uint8_t command = 1;			// 1でモードチェンジ
	uint16_t throttle = 0;			// なんでもよい
	
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;
	
	mavlink_msg_chaser_cmd_pack(system_id, component_id, &msg, command, state, throttle);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	Serial.write(buf, len);
}

// CHASER用スロットル操作コマンド送信
// uint16_t throttle	0-1000
void send_throttle_for_chaser_cmd(uint16_t throttle){
	uint8_t system_id = 20;			// 実績値20
	uint8_t component_id = 200;		// 実績値200
	uint8_t command = 2;			// 2でスロットル操作
	uint8_t state = 0;				// なんでもよい
	
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;
	
	mavlink_msg_chaser_cmd_pack(system_id, component_id, &msg, command, state, throttle);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	Serial.write(buf, len);
}





