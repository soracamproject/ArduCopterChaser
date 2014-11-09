// MESSAGE CHASER_DISTANCE PACKING

#define MAVLINK_MSG_ID_CHASER_DISTANCE 203

typedef struct __mavlink_chaser_distance_t
{
 float distance; ///< distance between copter and beacon
} mavlink_chaser_distance_t;

#define MAVLINK_MSG_ID_CHASER_DISTANCE_LEN 4
#define MAVLINK_MSG_ID_203_LEN 4

#define MAVLINK_MSG_ID_CHASER_DISTANCE_CRC 143
#define MAVLINK_MSG_ID_203_CRC 143



#define MAVLINK_MESSAGE_INFO_CHASER_DISTANCE { \
	"CHASER_DISTANCE", \
	1, \
	{  { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_chaser_distance_t, distance) }, \
         } \
}


/**
 * @brief Pack a chaser_distance message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param distance distance between copter and beacon
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chaser_distance_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_DISTANCE_LEN];
	_mav_put_float(buf, 0, distance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASER_DISTANCE_LEN);
#else
	mavlink_chaser_distance_t packet;
	packet.distance = distance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHASER_DISTANCE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CHASER_DISTANCE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHASER_DISTANCE_LEN, MAVLINK_MSG_ID_CHASER_DISTANCE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHASER_DISTANCE_LEN);
#endif
}

/**
 * @brief Pack a chaser_distance message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param distance distance between copter and beacon
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chaser_distance_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_DISTANCE_LEN];
	_mav_put_float(buf, 0, distance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASER_DISTANCE_LEN);
#else
	mavlink_chaser_distance_t packet;
	packet.distance = distance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHASER_DISTANCE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CHASER_DISTANCE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHASER_DISTANCE_LEN, MAVLINK_MSG_ID_CHASER_DISTANCE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHASER_DISTANCE_LEN);
#endif
}

/**
 * @brief Encode a chaser_distance struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param chaser_distance C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chaser_distance_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_chaser_distance_t* chaser_distance)
{
	return mavlink_msg_chaser_distance_pack(system_id, component_id, msg, chaser_distance->distance);
}

/**
 * @brief Encode a chaser_distance struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param chaser_distance C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chaser_distance_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_chaser_distance_t* chaser_distance)
{
	return mavlink_msg_chaser_distance_pack_chan(system_id, component_id, chan, msg, chaser_distance->distance);
}

/**
 * @brief Send a chaser_distance message
 * @param chan MAVLink channel to send the message
 *
 * @param distance distance between copter and beacon
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_chaser_distance_send(mavlink_channel_t chan, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_DISTANCE_LEN];
	_mav_put_float(buf, 0, distance);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_DISTANCE, buf, MAVLINK_MSG_ID_CHASER_DISTANCE_LEN, MAVLINK_MSG_ID_CHASER_DISTANCE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_DISTANCE, buf, MAVLINK_MSG_ID_CHASER_DISTANCE_LEN);
#endif
#else
	mavlink_chaser_distance_t packet;
	packet.distance = distance;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_DISTANCE, (const char *)&packet, MAVLINK_MSG_ID_CHASER_DISTANCE_LEN, MAVLINK_MSG_ID_CHASER_DISTANCE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_DISTANCE, (const char *)&packet, MAVLINK_MSG_ID_CHASER_DISTANCE_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_CHASER_DISTANCE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_chaser_distance_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, distance);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_DISTANCE, buf, MAVLINK_MSG_ID_CHASER_DISTANCE_LEN, MAVLINK_MSG_ID_CHASER_DISTANCE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_DISTANCE, buf, MAVLINK_MSG_ID_CHASER_DISTANCE_LEN);
#endif
#else
	mavlink_chaser_distance_t *packet = (mavlink_chaser_distance_t *)msgbuf;
	packet->distance = distance;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_DISTANCE, (const char *)packet, MAVLINK_MSG_ID_CHASER_DISTANCE_LEN, MAVLINK_MSG_ID_CHASER_DISTANCE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_DISTANCE, (const char *)packet, MAVLINK_MSG_ID_CHASER_DISTANCE_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE CHASER_DISTANCE UNPACKING


/**
 * @brief Get field distance from chaser_distance message
 *
 * @return distance between copter and beacon
 */
static inline float mavlink_msg_chaser_distance_get_distance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a chaser_distance message into a struct
 *
 * @param msg The message to decode
 * @param chaser_distance C-struct to decode the message contents into
 */
static inline void mavlink_msg_chaser_distance_decode(const mavlink_message_t* msg, mavlink_chaser_distance_t* chaser_distance)
{
#if MAVLINK_NEED_BYTE_SWAP
	chaser_distance->distance = mavlink_msg_chaser_distance_get_distance(msg);
#else
	memcpy(chaser_distance, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_CHASER_DISTANCE_LEN);
#endif
}
