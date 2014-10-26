/** charset=UTF-8 **/

#ifndef __BC_GPS_H__
#define __BC_GPS_H__

#include <BC_Common.h>
#include <FastSerial.h>

class BC_GPS
{
public:
	BC_GPS(FastSerial &gps_serial):
		_gps_serial(gps_serial)
	{}
	
	int32_t lat_data;
	int32_t lon_data;
	
	enum GPS_Status{
		NO_GPS = 0,             ///< No GPS connected/detected
		NO_FIX = 1,             ///< Receiving valid GPS messages but no lock
		GPS_OK_FIX_2D = 2,      ///< Receiving valid messages and 2D lock
		GPS_OK_FIX_3D = 3,      ///< Receiving valid messages and 3D lock
	};
	
	void init_gps();
	void get_gps_new_data();
	GPS_Status status() const { return GPS_status; }
	float ground_speed() const { return _ground_speed;}
	int32_t ground_course_cd() const { return _ground_course_cd; }
	
	
private:
	// GPS関連の変数
	int32_t		GPS_read[2];
	uint8_t		GPS_numSats;
	int16_t		GPS_altitude;				// 単位[m]
	float		_ground_speed;				// 単位[m/s]
	int32_t		_ground_course_cd;
	uint32_t	GPS_last_gps_time_ms;
	GPS_Status	GPS_status;
	uint16_t	GPS_hdop;					// horizontal dilution of precision in cm
	uint32_t	GPS_time_week_ms;              ///< GPS time (milliseconds from start of GPS week)
	uint16_t	GPS_time_week;                 ///< GPS week number
	uint32_t	_last_pos_time;
	uint32_t	_last_5hz_time;
    

	
	
	// UBLOX関連の変数（必要性を精査できていない）
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
		int32_t		horizontal_accuracy;
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
	uint8_t		_ck_a;
	uint8_t		_ck_b;
	
	// State machine state
	uint8_t		_step;
	uint8_t		_msg_id;
	uint16_t	_payload_length;
	uint16_t	_payload_counter;
	
	GPS_Status	next_fix;
	
	uint8_t		_class;
	
	// do we have new position information?
	bool		_new_position;
	
	// do we have new speed information?
	bool		_new_speed;
	
	uint8_t		_disable_counter;
	
	// Receive buffer
	union {
		ubx_nav_posllh		posllh;
		ubx_nav_status		status;
		ubx_nav_solution	solution;
		ubx_nav_velned		velned;
		uint8_t				bytes[];
	} _buffer;
	
	static const prog_char _initialisation[];
	

	
	void print_P(const char *str);
	void _update_checksum(uint8_t *data, uint8_t len, uint8_t &ck_a, uint8_t &ck_b);
	bool GPS_UBLOX_newFrame(uint8_t data);
	bool UBLOX_parse_gps(void);

protected:
	FastSerial	&_gps_serial;

};


#endif // __BC_GPS_H__
