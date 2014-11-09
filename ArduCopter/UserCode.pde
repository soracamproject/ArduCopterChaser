/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
    // put your 3.3Hz code here
	
	// 常に機体ステータスを送信
	gcs_send_message(MSG_CHASER_COPTER_STATUS);
	
	// Chaser_Stay時のみ機体位置を送信する
	if(chaser_state == CHASER_STAY){
		gcs_send_message(MSG_CHASER_DISTANCE);
	}
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif