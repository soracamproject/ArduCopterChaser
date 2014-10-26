/** charset=UTF-8 **/

#include "BC_GPS.h"
#include <FastSerial.h>
#include <Arduino.h>

//extern FastSerial gps_serial;

#define GPS_BAUD   38400

const prog_char BC_GPS::_initialisation[] PROGMEM = {                          // PROGMEM array must be outside any function !!!
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
	0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A //set rate to 5Hz
};




// *************************************************************************************************
// Helper function to allow serial output from a string help in program memory
//
// Using Serial.write(PSTR("$PUBX,41,1,0003,0001,xxxxxx,0*1E\r\n")); caused an error, the serial data was empty (all zeros) 
// but the packet length was correct. Possibly Serial.write() does not support being handed a pointer to a string in program memory
// *************************************************************************************************
void BC_GPS::print_P(const char *str){
  uint8_t val;
  while (true) {
    val=pgm_read_byte(str);
    if (!val) break;
    _gps_serial.write(val);
    str++;
  }
}


void BC_GPS::init_gps() {
	_gps_serial.begin(GPS_BAUD);
	delay(1000);
	
	uint32_t	init_speed[5] = {9600,19200,38400,57600,115200};
	
	//Set speed
	for(uint8_t i=0;i<5;i++){
		_gps_serial.begin(init_speed[i]);          // switch UART speed for sending SET BAUDRATE command (NMEA mode)
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
	_gps_serial.begin(GPS_BAUD);
	for(uint8_t i=0; i<sizeof(_initialisation); i++) {                        // send configuration data in UBX protocol
		_gps_serial.write(pgm_read_byte(_initialisation+i));
		delay(5);	//simulating a 38400baud pace (or less), otherwise commands are not accepted by the device. 
					//I found this delay essential for the V1 CN-06 GPS (The one without EEPROM)
					//Also essential is the bridging of pins 13- and 14 on the NEO-6M module	
	}
}


void BC_GPS::get_gps_new_data() {
	while (_gps_serial.available()) {
		if (GPS_UBLOX_newFrame(_gps_serial.read())) {
			// We have a valid GGA frame and we have lat and lon in GPS_read_lat and GPS_read_lon, apply moving average filter
			// this is a little bit tricky since the 1e7/deg precision easily overflow a long, so we apply the filter to the fractions
			// only, and strip the full degrees part. This means that we have to disable the filter if we are very close to a degree line
			
			// Think this line was in the wrong place. The way it used to be the lastframe_time was only updated when we had a (3D fix && we have 5 or more sats).
			// This stops the single led blink from indicating a good packet, and the double led blink from indicating a 2D fix
			//lastframe_time = millis();
			
			lat_data = GPS_read[LAT];
			lon_data = GPS_read[LON];
		}
	}
}


void BC_GPS::_update_checksum(uint8_t *data, uint8_t len, uint8_t &ck_a, uint8_t &ck_b) {
	while (len--) {
		ck_a += *data;
		ck_b += ck_a;
		data++;
	}
}


bool BC_GPS::GPS_UBLOX_newFrame(uint8_t data){
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
			if (UBLOX_parse_gps())  { parsed = true; }
        } //end switch
   return parsed;
}

bool BC_GPS::UBLOX_parse_gps(void){
	switch (_msg_id) {
		case MSG_POSLLH:
			_last_pos_time = _buffer.posllh.time;
			GPS_read[LON] = _buffer.posllh.longitude;
			GPS_read[LAT] = _buffer.posllh.latitude;
			GPS_altitude = _buffer.posllh.altitude_msl / 10 /100;      //alt in m
			GPS_status = next_fix;
			_new_position = true;
			break;
			
		case MSG_STATUS:
			if(_buffer.status.fix_status & NAV_STATUS_FIX_VALID) {
				if( _buffer.status.fix_type == FIX_3D) {
					next_fix = GPS_OK_FIX_3D;
				}else if (_buffer.status.fix_type == FIX_2D) {
					next_fix = GPS_OK_FIX_2D;
				}else{
					next_fix = NO_FIX;
					GPS_status = NO_FIX;
				}
			}else{
				next_fix = NO_FIX;
				GPS_status = NO_FIX;
			}
			break;
			
		case MSG_SOL:
			if(_buffer.solution.fix_status & NAV_STATUS_FIX_VALID){
				if( _buffer.solution.fix_type == FIX_3D) {
					next_fix = GPS_OK_FIX_3D;
				}else if (_buffer.solution.fix_type == FIX_2D) {
					next_fix = GPS_OK_FIX_2D;
				}else{
					next_fix = NO_FIX;
					GPS_status = NO_FIX;
				}
			}else{
				next_fix = NO_FIX;
				GPS_status = NO_FIX;
			}
			GPS_numSats    = _buffer.solution.satellites;
			GPS_hdop        = _buffer.solution.position_DOP;
			if (next_fix >= GPS_OK_FIX_2D) {
				GPS_last_gps_time_ms = millis();
				if (GPS_time_week == _buffer.solution.week &&
					GPS_time_week_ms + 200 == _buffer.solution.time) {
					// we got a 5Hz update. This relies on the way
					// that uBlox gives timestamps that are always
					// multiples of 200 for 5Hz
					_last_5hz_time = GPS_last_gps_time_ms;
				}
				GPS_time_week_ms    = _buffer.solution.time;
				GPS_time_week       = _buffer.solution.week;
			}
			break;
			
		case MSG_VELNED:
			_ground_speed = _buffer.velned.speed_2d*0.01f;			// m/s
			_ground_course_cd = _buffer.velned.heading_2d / 1000;	// Heading 2D deg * 100000 rescaled to deg * 100
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
/*
void BC_GPS::blink_update() {
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
*/

