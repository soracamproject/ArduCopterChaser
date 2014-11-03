static void check_input_msg(void)
{
	// 変数準備
	mavlink_message_t msg;
	mavlink_status_t status;
	status.packet_rx_drop_count = 0;
	
	static uint8_t flag_led_debug = 0;
	
	// 受信可能バイトを取得
	int16_t nbytes = (int16_t)xbee_serial.available();
	if (nbytes == -1){
		nbytes = 0;
	}
	
	for (uint16_t i=0; i<nbytes; i++)
	{
		// データ読み込み
		uint8_t c = (uint8_t)xbee_serial.read();
		
		// 新しいメッセージかどうかを判断する
		if (mavlink_parse_char(0, c, &msg, &status)) {
			// とりあえずコメントアウト
			// // we exclude radio packets to make it possible to use the
			// // CLI over the radio
			//if (msg.msgid != MAVLINK_MSG_ID_RADIO && msg.msgid != MAVLINK_MSG_ID_RADIO_STATUS) {
			//    mavlink_active |= (1U<<chan);
			//}
			handleMessage(&msg);
		}
	}
}

void handleMessage(mavlink_message_t* msg){
	//uint8_t result = MAV_RESULT_FAILED;         // assume failure.  Each messages id is responsible for return ACK or NAK if required
	static uint16_t msg_count = 0;
	console.print("message_received");
	console.println(msg->msgid);
	static uint8_t count_not_chaser = 0;;
	
	switch (msg->msgid) {
		case MAVLINK_MSG_ID_CHASER_STATUS:
			mavlink_chaser_status_t packet;
			mavlink_msg_chaser_status_decode(msg, &packet);
			
			update_copter_status(packet);
			//copter_mode    = packet.control_mode;
			//copter_state   = packet.chaser_state;
			//copter_num_sat = packet.num_sat;
			//copter_armed   = packet.armed;
			
			break;
	}	// msgidのスイッチの中括弧とじ
}	// handleMessage関数の中括弧とじ

static void update_copter_status(mavlink_chaser_status_t& packet){
	static uint8_t mode_last;
	static uint8_t mode_count;
	static uint8_t state_last;
	static uint8_t state_count;
	static uint8_t num_sat_last;
	static uint8_t num_sat_count;
	static uint8_t armed_last;
	static uint8_t armed_count;
	
	// 値が変化した後所定回数以上続いたら値を更新する
	// 力技、とりあえず
	
	// control_mode
	if(packet.control_mode != mode_last){
		mode_count = 1;
	} else {
		if(mode_count > 0 && ++mode_count >= STATUS_UPDATE_NUM){
			copter_mode = packet.control_mode;
			mode_count = 0;
		}
	}
	mode_last = packet.control_mode;
	
	// chaser_state
	if(packet.chaser_state != state_last){
		state_count = 1;
	} else {
		if(state_count > 0 && ++state_count >= STATUS_UPDATE_NUM){
			copter_state = packet.chaser_state;
			state_count = 0;
		}
	}
	state_last = packet.chaser_state;
	
	// num_sat
	if(packet.num_sat != num_sat_last){
		num_sat_count = 1;
	} else {
		if(num_sat_count > 0 && ++num_sat_count >= STATUS_UPDATE_NUM){
			copter_num_sat = packet.num_sat;
			num_sat_count = 0;
		}
	}
	num_sat_last = packet.num_sat;
	
	// armed
	if(packet.armed != armed_last){
		armed_count = 1;
	} else {
		if(armed_count > 0 && ++armed_count >= STATUS_UPDATE_NUM){
			copter_armed = packet.armed;
			armed_count = 0;
		}
	}
	armed_last = packet.armed;
}

