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
	
	switch (msg->msgid) {
		case MAVLINK_MSG_ID_CHASER_CMD:
			mavlink_chaser_cmd_t packet;
			mavlink_msg_chaser_cmd_decode(msg, &packet);
			
			if(packet.command == 4){
				copter_id[debug_count] = packet.p2;
				copter_time_received[debug_count] = packet.p3;
				beacon_time_received[debug_count] = millis();
				debug_count++;
				debug_send_flag = true;
			}
			break;
	}	// msgidのスイッチの中括弧とじ
}	// handleMessage関数の中括弧とじ


