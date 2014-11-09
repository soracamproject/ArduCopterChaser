// MESSAGE CHASER_RECALC_OFFSET PACKING

#define MAVLINK_MSG_ID_CHASER_RECALC_OFFSET 204

typedef struct __mavlink_chaser_recalc_offset_t
{
 float offset_x; ///< copter beacon position offset x
 float offset_y; ///< copter beacon position offset y
} mavlink_chaser_recalc_offset_t;

#define MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_LEN 8
#define MAVLINK_MSG_ID_204_LEN 8

#define MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_CRC 56
#define MAVLINK_MSG_ID_204_CRC 56



#define MAVLINK_MESSAGE_INFO_CHASER_RECALC_OFFSET { \
	"CHASER_RECALC_OFFSET", \
	2, \
	{  { "offset_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_chaser_recalc_offset_t, offset_x) }, \
         { "offset_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_chaser_recalc_offset_t, offset_y) }, \
         } \
}


/**
 * @brief Pack a chaser_recalc_offset message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param offset_x copter beacon position offset x
 * @param offset_y copter beacon position offset y
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chaser_recalc_offset_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float offset_x, float offset_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_LEN];
	_mav_put_float(buf, 0, offset_x);
	_mav_put_float(buf, 4, offset_y);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_LEN);
#else
	mavlink_chaser_recalc_offset_t packet;
	packet.offset_x = offset_x;
	packet.offset_y = offset_y;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CHASER_RECALC_OFFSET;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_LEN, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_LEN);
#endif
}

/**
 * @brief Pack a chaser_recalc_offset message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param offset_x copter beacon position offset x
 * @param offset_y copter beacon position offset y
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chaser_recalc_offset_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float offset_x,float offset_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_LEN];
	_mav_put_float(buf, 0, offset_x);
	_mav_put_float(buf, 4, offset_y);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_LEN);
#else
	mavlink_chaser_recalc_offset_t packet;
	packet.offset_x = offset_x;
	packet.offset_y = offset_y;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CHASER_RECALC_OFFSET;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_LEN, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_LEN);
#endif
}

/**
 * @brief Encode a chaser_recalc_offset struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param chaser_recalc_offset C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chaser_recalc_offset_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_chaser_recalc_offset_t* chaser_recalc_offset)
{
	return mavlink_msg_chaser_recalc_offset_pack(system_id, component_id, msg, chaser_recalc_offset->offset_x, chaser_recalc_offset->offset_y);
}

/**
 * @brief Encode a chaser_recalc_offset struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param chaser_recalc_offset C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chaser_recalc_offset_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_chaser_recalc_offset_t* chaser_recalc_offset)
{
	return mavlink_msg_chaser_recalc_offset_pack_chan(system_id, component_id, chan, msg, chaser_recalc_offset->offset_x, chaser_recalc_offset->offset_y);
}

/**
 * @brief Send a chaser_recalc_offset message
 * @param chan MAVLink channel to send the message
 *
 * @param offset_x copter beacon position offset x
 * @param offset_y copter beacon position offset y
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_chaser_recalc_offset_send(mavlink_channel_t chan, float offset_x, float offset_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_LEN];
	_mav_put_float(buf, 0, offset_x);
	_mav_put_float(buf, 4, offset_y);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET, buf, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_LEN, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET, buf, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_LEN);
#endif
#else
	mavlink_chaser_recalc_offset_t packet;
	packet.offset_x = offset_x;
	packet.offset_y = offset_y;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET, (const char *)&packet, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_LEN, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET, (const char *)&packet, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_chaser_recalc_offset_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float offset_x, float offset_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, offset_x);
	_mav_put_float(buf, 4, offset_y);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET, buf, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_LEN, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET, buf, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_LEN);
#endif
#else
	mavlink_chaser_recalc_offset_t *packet = (mavlink_chaser_recalc_offset_t *)msgbuf;
	packet->offset_x = offset_x;
	packet->offset_y = offset_y;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET, (const char *)packet, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_LEN, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET, (const char *)packet, MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE CHASER_RECALC_OFFSET UNPACKING


/**
 * @brief Get field offset_x from chaser_recalc_offset message
 *
 * @return copter beacon position offset x
 */
static inline float mavlink_msg_chaser_recalc_offset_get_offset_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field offset_y from chaser_recalc_offset message
 *
 * @return copter beacon position offset y
 */
static inline float mavlink_msg_chaser_recalc_offset_get_offset_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a chaser_recalc_offset message into a struct
 *
 * @param msg The message to decode
 * @param chaser_recalc_offset C-struct to decode the message contents into
 */
static inline void mavlink_msg_chaser_recalc_offset_decode(const mavlink_message_t* msg, mavlink_chaser_recalc_offset_t* chaser_recalc_offset)
{
#if MAVLINK_NEED_BYTE_SWAP
	chaser_recalc_offset->offset_x = mavlink_msg_chaser_recalc_offset_get_offset_x(msg);
	chaser_recalc_offset->offset_y = mavlink_msg_chaser_recalc_offset_get_offset_y(msg);
#else
	memcpy(chaser_recalc_offset, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_CHASER_RECALC_OFFSET_LEN);
#endif
}
