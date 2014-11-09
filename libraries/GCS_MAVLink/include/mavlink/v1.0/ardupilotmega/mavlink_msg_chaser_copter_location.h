// MESSAGE CHASER_COPTER_LOCATION PACKING

#define MAVLINK_MSG_ID_CHASER_COPTER_LOCATION 203

typedef struct __mavlink_chaser_copter_location_t
{
 float pos_x; ///< copter position x
 float pos_y; ///< copter position y
} mavlink_chaser_copter_location_t;

#define MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_LEN 8
#define MAVLINK_MSG_ID_203_LEN 8

#define MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_CRC 246
#define MAVLINK_MSG_ID_203_CRC 246



#define MAVLINK_MESSAGE_INFO_CHASER_COPTER_LOCATION { \
	"CHASER_COPTER_LOCATION", \
	2, \
	{  { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_chaser_copter_location_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_chaser_copter_location_t, pos_y) }, \
         } \
}


/**
 * @brief Pack a chaser_copter_location message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pos_x copter position x
 * @param pos_y copter position y
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chaser_copter_location_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float pos_x, float pos_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_LEN];
	_mav_put_float(buf, 0, pos_x);
	_mav_put_float(buf, 4, pos_y);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_LEN);
#else
	mavlink_chaser_copter_location_t packet;
	packet.pos_x = pos_x;
	packet.pos_y = pos_y;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CHASER_COPTER_LOCATION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_LEN, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_LEN);
#endif
}

/**
 * @brief Pack a chaser_copter_location message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pos_x copter position x
 * @param pos_y copter position y
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chaser_copter_location_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float pos_x,float pos_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_LEN];
	_mav_put_float(buf, 0, pos_x);
	_mav_put_float(buf, 4, pos_y);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_LEN);
#else
	mavlink_chaser_copter_location_t packet;
	packet.pos_x = pos_x;
	packet.pos_y = pos_y;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CHASER_COPTER_LOCATION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_LEN, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_LEN);
#endif
}

/**
 * @brief Encode a chaser_copter_location struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param chaser_copter_location C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chaser_copter_location_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_chaser_copter_location_t* chaser_copter_location)
{
	return mavlink_msg_chaser_copter_location_pack(system_id, component_id, msg, chaser_copter_location->pos_x, chaser_copter_location->pos_y);
}

/**
 * @brief Encode a chaser_copter_location struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param chaser_copter_location C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chaser_copter_location_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_chaser_copter_location_t* chaser_copter_location)
{
	return mavlink_msg_chaser_copter_location_pack_chan(system_id, component_id, chan, msg, chaser_copter_location->pos_x, chaser_copter_location->pos_y);
}

/**
 * @brief Send a chaser_copter_location message
 * @param chan MAVLink channel to send the message
 *
 * @param pos_x copter position x
 * @param pos_y copter position y
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_chaser_copter_location_send(mavlink_channel_t chan, float pos_x, float pos_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_LEN];
	_mav_put_float(buf, 0, pos_x);
	_mav_put_float(buf, 4, pos_y);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION, buf, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_LEN, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION, buf, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_LEN);
#endif
#else
	mavlink_chaser_copter_location_t packet;
	packet.pos_x = pos_x;
	packet.pos_y = pos_y;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION, (const char *)&packet, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_LEN, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION, (const char *)&packet, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_chaser_copter_location_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float pos_x, float pos_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, pos_x);
	_mav_put_float(buf, 4, pos_y);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION, buf, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_LEN, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION, buf, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_LEN);
#endif
#else
	mavlink_chaser_copter_location_t *packet = (mavlink_chaser_copter_location_t *)msgbuf;
	packet->pos_x = pos_x;
	packet->pos_y = pos_y;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION, (const char *)packet, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_LEN, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION, (const char *)packet, MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE CHASER_COPTER_LOCATION UNPACKING


/**
 * @brief Get field pos_x from chaser_copter_location message
 *
 * @return copter position x
 */
static inline float mavlink_msg_chaser_copter_location_get_pos_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pos_y from chaser_copter_location message
 *
 * @return copter position y
 */
static inline float mavlink_msg_chaser_copter_location_get_pos_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a chaser_copter_location message into a struct
 *
 * @param msg The message to decode
 * @param chaser_copter_location C-struct to decode the message contents into
 */
static inline void mavlink_msg_chaser_copter_location_decode(const mavlink_message_t* msg, mavlink_chaser_copter_location_t* chaser_copter_location)
{
#if MAVLINK_NEED_BYTE_SWAP
	chaser_copter_location->pos_x = mavlink_msg_chaser_copter_location_get_pos_x(msg);
	chaser_copter_location->pos_y = mavlink_msg_chaser_copter_location_get_pos_y(msg);
#else
	memcpy(chaser_copter_location, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_CHASER_COPTER_LOCATION_LEN);
#endif
}
