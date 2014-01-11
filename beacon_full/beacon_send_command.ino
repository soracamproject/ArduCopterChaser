// アームコマンド送信
void send_arm_cmd(){
	uint8_t system_id = 20;			// 実績値20
	uint8_t component_id = 200;		// 実績値200
	uint8_t target_system = 1;		// 1が必要（MISSION PLANNERのFull Param ListでSYSで検索）
	uint8_t target_component = MAV_COMP_ID_SYSTEM_CONTROL;
	uint16_t command = MAV_CMD_COMPONENT_ARM_DISARM;
	uint8_t confirmation = 1;		// 実績値1
	float param1 = 1.0f;			// ARM(1),DISARM(0)
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

// INITコマンド送信(CHASERモードに入れる)
void send_init_cmd(){
	uint8_t system_id = 20;			// 実績値20
	uint8_t component_id = 200;		// 実績値200
	uint8_t command = 1;			// とりあえず1で実行ということにする
	uint8_t mode = CHASER_INIT;
	
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;
	
	mavlink_msg_chaser_cmd_pack(system_id, component_id, &msg, command, mode);
	
	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	Serial.write(buf, len);
}

// READYコマンド送信
void send_ready_cmd(){
	uint8_t system_id = 20;			// 実績値20
	uint8_t component_id = 200;		// 実績値200
	uint8_t command = 1;			// とりあえず1で実行ということにする
	uint8_t mode = CHASER_READY;
	
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;
	
	mavlink_msg_chaser_cmd_pack(system_id, component_id, &msg, command, mode);
	
	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	Serial.write(buf, len);
}


// テイクオフコマンド送信
void send_takeoff_cmd(){
	uint8_t system_id = 20;			// 実績値20
	uint8_t component_id = 200;		// 実績値200
	uint8_t command = 1;			// とりあえず1で実行ということにする
	uint8_t mode = CHASER_TAKEOFF;
	
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;
	
	mavlink_msg_chaser_cmd_pack(system_id, component_id, &msg, command, mode);
	
	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	Serial.write(buf, len);
}

// ランディングコマンド送信
void send_land_cmd(){
	uint8_t system_id = 20;			// 実績値20
	uint8_t component_id = 200;		// 実績値200
	uint8_t command = 1;			// とりあえず1で実行ということにする
	uint8_t mode = CHASER_EM_LAND;	// 本当はCHASER_LAND
	
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;
	
	mavlink_msg_chaser_cmd_pack(system_id, component_id, &msg, command, mode);
	
	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	Serial.write(buf, len);
}



/////////////////////////////////////////////////////////
// 以下、過去のものや参考物
/////////////////////////////////////////////////////////

/*
// CHASERコマンド送信
void send_chaser_cmd(int32_t lat, int32_t lon, int16_t alt){
	uint8_t system_id = 20;			// 実績値20
	uint8_t component_id = 200;		// 実績値200
	
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;
	
	mavlink_msg_chaser_cmd_pack(system_id, component_id, &msg, lat, lon, alt);
	
	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	Serial.write(buf, len);
}
*/