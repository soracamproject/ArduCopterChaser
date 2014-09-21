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
			//handleMessage(&msg);
			if(flag_led_debug == 0)
			{
				control_led(0,0,0,1);
				flag_led_debug = 1;
			} else {
				control_led(0,0,0,-1);
				flag_led_debug = 0;
			}
		}
	}
}