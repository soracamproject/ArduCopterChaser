// MESSAGE CHASER_BEACON_LOCATION PACKING

#define MAVLINK_MSG_ID_CHASER_BEACON_LOCATION 201

typedef struct __mavlink_chaser_beacon_location_t
{
 int32_t lat; ///< latitude (degree * 10^7)
 int32_t lon; ///< longitude (degree * 10^7)
 int32_t pressure; ///< pressure (*100 mbar = Pa)
} mavlink_chaser_beacon_location_t;

#define MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_LEN 12
#define MAVLINK_MSG_ID_201_LEN 12

#define MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_CRC 103
#define MAVLINK_MSG_ID_201_CRC 103



#define MAVLINK_MESSAGE_INFO_CHASER_BEACON_LOCATION { \
	"CHASER_BEACON_LOCATION", \
	3, \
	{  { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_chaser_beacon_location_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_chaser_beacon_location_t, lon) }, \
         { "pressure", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_chaser_beacon_location_t, pressure) }, \
         } \
}


/**
 * @brief Pack a chaser_beacon_location message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat latitude (degree * 10^7)
 * @param lon longitude (degree * 10^7)
 * @param pressure pressure (*100 mbar = Pa)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chaser_beacon_location_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int32_t lat, int32_t lon, int32_t pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_LEN];
	_mav_put_int32_t(buf, 0, lat);
	_mav_put_int32_t(buf, 4, lon);
	_mav_put_int32_t(buf, 8, pressure);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_LEN);
#else
	mavlink_chaser_beacon_location_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.pressure = pressure;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CHASER_BEACON_LOCATION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_LEN, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_LEN);
#endif
}

/**
 * @brief Pack a chaser_beacon_location message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lat latitude (degree * 10^7)
 * @param lon longitude (degree * 10^7)
 * @param pressure pressure (*100 mbar = Pa)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chaser_beacon_location_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int32_t lat,int32_t lon,int32_t pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_LEN];
	_mav_put_int32_t(buf, 0, lat);
	_mav_put_int32_t(buf, 4, lon);
	_mav_put_int32_t(buf, 8, pressure);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_LEN);
#else
	mavlink_chaser_beacon_location_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.pressure = pressure;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CHASER_BEACON_LOCATION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_LEN, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_LEN);
#endif
}

/**
 * @brief Encode a chaser_beacon_location struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param chaser_beacon_location C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chaser_beacon_location_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_chaser_beacon_location_t* chaser_beacon_location)
{
	return mavlink_msg_chaser_beacon_location_pack(system_id, component_id, msg, chaser_beacon_location->lat, chaser_beacon_location->lon, chaser_beacon_location->pressure);
}

/**
 * @brief Encode a chaser_beacon_location struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param chaser_beacon_location C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chaser_beacon_location_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_chaser_beacon_location_t* chaser_beacon_location)
{
	return mavlink_msg_chaser_beacon_location_pack_chan(system_id, component_id, chan, msg, chaser_beacon_location->lat, chaser_beacon_location->lon, chaser_beacon_location->pressure);
}

/**
 * @brief Send a chaser_beacon_location message
 * @param chan MAVLink channel to send the message
 *
 * @param lat latitude (degree * 10^7)
 * @param lon longitude (degree * 10^7)
 * @param pressure pressure (*100 mbar = Pa)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_chaser_beacon_location_send(mavlink_channel_t chan, int32_t lat, int32_t lon, int32_t pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_LEN];
	_mav_put_int32_t(buf, 0, lat);
	_mav_put_int32_t(buf, 4, lon);
	_mav_put_int32_t(buf, 8, pressure);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION, buf, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_LEN, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION, buf, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_LEN);
#endif
#else
	mavlink_chaser_beacon_location_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.pressure = pressure;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION, (const char *)&packet, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_LEN, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION, (const char *)&packet, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_chaser_beacon_location_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t lat, int32_t lon, int32_t pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int32_t(buf, 0, lat);
	_mav_put_int32_t(buf, 4, lon);
	_mav_put_int32_t(buf, 8, pressure);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION, buf, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_LEN, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION, buf, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_LEN);
#endif
#else
	mavlink_chaser_beacon_location_t *packet = (mavlink_chaser_beacon_location_t *)msgbuf;
	packet->lat = lat;
	packet->lon = lon;
	packet->pressure = pressure;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION, (const char *)packet, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_LEN, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION, (const char *)packet, MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE CHASER_BEACON_LOCATION UNPACKING


/**
 * @brief Get field lat from chaser_beacon_location message
 *
 * @return latitude (degree * 10^7)
 */
static inline int32_t mavlink_msg_chaser_beacon_location_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field lon from chaser_beacon_location message
 *
 * @return longitude (degree * 10^7)
 */
static inline int32_t mavlink_msg_chaser_beacon_location_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field pressure from chaser_beacon_location message
 *
 * @return pressure (*100 mbar = Pa)
 */
static inline int32_t mavlink_msg_chaser_beacon_location_get_pressure(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Decode a chaser_beacon_location message into a struct
 *
 * @param msg The message to decode
 * @param chaser_beacon_location C-struct to decode the message contents into
 */
static inline void mavlink_msg_chaser_beacon_location_decode(const mavlink_message_t* msg, mavlink_chaser_beacon_location_t* chaser_beacon_location)
{
#if MAVLINK_NEED_BYTE_SWAP
	chaser_beacon_location->lat = mavlink_msg_chaser_beacon_location_get_lat(msg);
	chaser_beacon_location->lon = mavlink_msg_chaser_beacon_location_get_lon(msg);
	chaser_beacon_location->pressure = mavlink_msg_chaser_beacon_location_get_pressure(msg);
#else
	memcpy(chaser_beacon_location, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_CHASER_BEACON_LOCATION_LEN);
#endif
}
