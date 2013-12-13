
// *************************************************************************************************
// ごちゃまぜで定義
// *************************************************************************************************
#define GPS_SERIAL					0
#define GPS_BAUD					38400


// *************************************************************************************************
// Helper function to allow serial output from a string help in program memory
//
// Using Serial.write(PSTR("$PUBX,41,1,0003,0001,xxxxxx,0*1E\r\n")); caused an error, the serial data was empty (all zeros) 
// but the packet length was correct. Possibly Serial.write() does not support being handed a pointer to a string in program memory
// *************************************************************************************************
void print_P(const char *str)
{
  uint8_t val;
  while (true) {
    val=pgm_read_byte(str);
    if (!val) break;
    Serial.write(val);
    str++;
  }
}


// *************************************************************************************************
// GPS取得関連
// *************************************************************************************************
uint32_t init_speed[5] = {9600,19200,38400,57600,115200};

prog_char UBLOX_INIT[] PROGMEM = {                          // PROGMEM array must be outside any function !!!
	0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x05,0x00,0xFF,0x19,                            //disable all default NMEA messages
	0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x03,0x00,0xFD,0x15,
	0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0x00,0xFB,0x11,
	0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x00,0x00,0xFA,0x0F,
	0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x02,0x00,0xFC,0x13,
	0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x04,0x00,0xFE,0x17,
	0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x02,0x01,0x0E,0x47,                            //set POSLLH MSG rate
	0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x03,0x01,0x0F,0x49,                            //set STATUS MSG rate
	0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x06,0x01,0x12,0x4F,                            //set SOL MSG rate
	0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x12,0x01,0x1E,0x67,                            //set VELNED MSG rate
	0xB5,0x62,0x06,0x16,0x08,0x00,0x03,0x07,0x03,0x00,0x51,0x08,0x00,0x00,0x8A,0x41,   //set WAAS to EGNOS
	0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A //set rate to 5Hz
};


void GPS_SerialInit() {
	Serial.begin(GPS_BAUD);  
	delay(1000);
	//Set speed
	for(uint8_t i=0;i<5;i++){
		Serial.begin(init_speed[i]);          // switch UART speed for sending SET BAUDRATE command (NMEA mode)
		#if (GPS_BAUD==19200)
			print_P(PSTR("$PUBX,41,1,0003,0001,19200,0*23\r\n"));     // 19200 baud - minimal speed for 5Hz update rate
		#endif  
		#if (GPS_BAUD==38400)
			print_P(PSTR("$PUBX,41,1,0003,0001,38400,0*26\r\n"));     // 38400 baud
		#endif  
		#if (GPS_BAUD==57600)
			print_P(PSTR("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));     // 57600 baud
		#endif  
		#if (GPS_BAUD==115200)
			print_P(PSTR("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));    // 115200 baud
		#endif  
		delay(300);		//Wait for init 
	}
	delay(200);
	Serial.begin(GPS_BAUD);  
	for(uint8_t i=0; i<sizeof(UBLOX_INIT); i++) {                        // send configuration data in UBX protocol
		Serial.write(pgm_read_byte(UBLOX_INIT+i));
		delay(5);	//simulating a 38400baud pace (or less), otherwise commands are not accepted by the device. 
					//I found this delay essential for the V1 CN-06 GPS (The one without EEPROM)
					//Also essential is the bridging of pins 13- and 14 on the NEO-6M module	
	}
}


// **************************************************************************************
// UBLOX関連の変数
// **************************************************************************************

struct ubx_header {
	uint8_t preamble1;
	uint8_t preamble2;
	uint8_t msg_class;
	uint8_t msg_id;
	uint16_t length;
};

struct ubx_nav_posllh {
	uint32_t	time;				// GPS msToW
	int32_t		longitude;
	int32_t		latitude;
	int32_t		altitude_ellipsoid;
	int32_t		altitude_msl;
	int32_t	horizontal_accuracy;
	uint32_t	vertical_accuracy;
};
    struct ubx_nav_status {
        uint32_t	time;				// GPS msToW
        uint8_t		fix_type;
        uint8_t		fix_status;
        uint8_t		differential_status;
        uint8_t		res;
        uint32_t	time_to_first_fix;
        uint32_t	uptime;				// milliseconds
    };
    struct ubx_nav_solution {
        uint32_t	time;
        int32_t		time_nsec;
        int16_t		week;
        uint8_t		fix_type;
        uint8_t		fix_status;
        int32_t		ecef_x;
        int32_t		ecef_y;
        int32_t		ecef_z;
        uint32_t	position_accuracy_3d;
        int32_t		ecef_x_velocity;
        int32_t		ecef_y_velocity;
        int32_t		ecef_z_velocity;
        uint32_t	speed_accuracy;
        uint16_t	position_DOP;
        uint8_t		res;
        uint8_t		satellites;
        uint32_t	res2;
    };
    struct ubx_nav_velned {
        uint32_t	time;				// GPS msToW
        int32_t		ned_north;
        int32_t		ned_east;
        int32_t		ned_down;
        uint32_t	speed_3d;
        uint32_t	speed_2d;
        int32_t		heading_2d;
        uint32_t	speed_accuracy;
        uint32_t	heading_accuracy;
    };

    enum ubs_protocol_bytes {
        PREAMBLE1 = 0xb5,
        PREAMBLE2 = 0x62,
        CLASS_NAV = 0x01,
        CLASS_ACK = 0x05,
        CLASS_CFG = 0x06,
		MSG_ACK_NACK = 0x00,
		MSG_ACK_ACK = 0x01,
        MSG_POSLLH = 0x2,
        MSG_STATUS = 0x3,
        MSG_SOL = 0x6,
        MSG_VELNED = 0x12,
        MSG_CFG_PRT = 0x00,
        MSG_CFG_RATE = 0x08,
        MSG_CFG_SET_RATE = 0x01,
		MSG_CFG_NAV_SETTINGS = 0x24
    };
    enum ubs_nav_fix_type {
        FIX_NONE = 0,
        FIX_DEAD_RECKONING = 1,
        FIX_2D = 2,
        FIX_3D = 3,
        FIX_GPS_DEAD_RECKONING = 4,
        FIX_TIME = 5
    };
    enum ubx_nav_status_bits {
        NAV_STATUS_FIX_VALID = 1
    };

    // Packet checksum accumulators
    static uint8_t		_ck_a;
    static uint8_t		_ck_b;

    // State machine state
    static uint8_t		_step;
    static uint8_t		_msg_id;
    static uint16_t	_payload_length;
    static uint16_t	_payload_counter;

    static bool        next_fix;
    
    static uint8_t     _class;

	// do we have new position information?
	static bool		_new_position;

	// do we have new speed information?
	static bool		_new_speed;

	static uint8_t	    _disable_counter;

    // Receive buffer
    static union {
        ubx_nav_posllh		posllh;
        ubx_nav_status		status;
        ubx_nav_solution	solution;
        ubx_nav_velned		velned;
        uint8_t	bytes[];
    } _buffer;



// **************************************************************************************
// UBLOX関連の関数
// **************************************************************************************
void _update_checksum(uint8_t *data, uint8_t len, uint8_t &ck_a, uint8_t &ck_b) {
	while (len--) {
		ck_a += *data;
		ck_b += ck_a;
		data++;
	}
}

bool GPS_UBLOX_newFrame(uint8_t data)
{
       bool parsed = false;

        switch(_step) {

        case 1:
            if (PREAMBLE2 == data) {
                _step++;
                break;
            }
            _step = 0;
        case 0:
            if(PREAMBLE1 == data) _step++;
            break;

        case 2:
            _step++;
	    _class = data;
	    _ck_b = _ck_a = data;			// reset the checksum accumulators
            break;
        case 3:
            _step++;
            _ck_b += (_ck_a += data);			// checksum byte
            _msg_id = data;
            break;
        case 4:
            _step++;
            _ck_b += (_ck_a += data);			// checksum byte
            _payload_length = data;				// payload length low byte
            break;
        case 5:
            _step++;
            _ck_b += (_ck_a += data);			// checksum byte

            _payload_length += (uint16_t)(data<<8);
			if (_payload_length > 512) {
				_payload_length = 0;
				_step = 0;
			}
            _payload_counter = 0;				// prepare to receive payload
            break;
        case 6:
            _ck_b += (_ck_a += data);			// checksum byte
			if (_payload_counter < sizeof(_buffer)) {
				_buffer.bytes[_payload_counter] = data;
			}
            if (++_payload_counter == _payload_length)
                _step++;
            break;
        case 7:
            _step++;
            if (_ck_a != data) _step = 0;						// bad checksum
            break;
        case 8:
            _step = 0;
            if (_ck_b != data)  break; 							// bad checksum
			//GPS_read[LAT] = GPS_debug;		//デバッグ用
			if (UBLOX_parse_gps())  { parsed = true; }
        } //end switch
   return parsed;
}

bool UBLOX_parse_gps(void){
	switch (_msg_id) {
		case MSG_POSLLH:
			GPS_debug++;
			GPS_time = _buffer.posllh.time;
			GPS_read[LON] = _buffer.posllh.longitude;
			GPS_read[LAT] = _buffer.posllh.latitude;
			//GPS_read[LAT] = GPS_debug;
			GPS_altitude = _buffer.posllh.altitude_msl / 10 /100;      //alt in m
			GPS_3dfix = next_fix;
			_new_position = true;
			break;
		case MSG_STATUS:
			next_fix = (_buffer.status.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.status.fix_type == FIX_3D);
			if (!next_fix) GPS_3dfix = false;
			break;
		case MSG_SOL:
			next_fix = (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D);
			if (!next_fix) GPS_3dfix = false;
				GPS_numSats	= _buffer.solution.satellites;
			break;
		case MSG_VELNED:
		GPS_ground_speed = _buffer.velned.speed_2d;				// cm/s
			GPS_ground_ground_course = (uint16_t)(_buffer.velned.heading_2d / 10000);	// Heading 2D deg * 100000 rescaled to deg * 10
			_new_speed = true;
			break;
		default:
			return false;
	}
	
	// we only return true when we get new position and speed data
	// this ensures we don't use stale data
	if (_new_position && _new_speed) {
		_new_speed = _new_position = false;
		return true;
	}
	return false;
}




// **************************************************************************************
// LED点滅関連
// **************************************************************************************
void blink_update() {
	uint32_t now = millis();
	
	if(_statusled_timer < now) {
		if(lastframe_time+5000 < now) {
			// no gps communication  
			_statusled_state = !_statusled_state;
			digitalWrite(13, _statusled_state ? HIGH : LOW);   // set the LED off
			_statusled_timer = now + 1000;
			return;
		}
	}
	
	if(_statusled_blinks==0) {	
		if(GPS_3dfix == 1)
			_statusled_blinks=3;
	} else if(GPS_2dfix == 1) {
		_statusled_blinks=2;
	} else {
		_statusled_blinks=1;      
	}
	
	if(_statusled_state) {
		_statusled_blinks--;
		_statusled_state = false;
		_statusled_timer = now + ((_statusled_blinks>0) ? BLINK_INTERVAL : 1000);
		digitalWrite(13, LOW);   // set the LED off
	} else {
		_statusled_state = true;
		_statusled_timer = now + BLINK_INTERVAL;
		digitalWrite(13, HIGH);   // set the LED on
	}
}

