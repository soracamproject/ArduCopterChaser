// MESSAGE CHASER_STATUS PACKING

#define MAVLINK_MSG_ID_CHASER_STATUS 202

typedef struct __mavlink_chaser_status_t
{
 uint8_t control_mode; ///< control mode
 uint8_t chaser_state; ///< chaser state
 uint8_t num_sat; ///< GPS number of satellite
 uint8_t armed; ///< armed or disarmed flag
} mavlink_chaser_status_t;

#define MAVLINK_MSG_ID_CHASER_STATUS_LEN 4
#define MAVLINK_MSG_ID_202_LEN 4

#define MAVLINK_MSG_ID_CHASER_STATUS_CRC 180
#define MAVLINK_MSG_ID_202_CRC 180



#define MAVLINK_MESSAGE_INFO_CHASER_STATUS { \
	"CHASER_STATUS", \
	4, \
	{  { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_chaser_status_t, control_mode) }, \
         { "chaser_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_chaser_status_t, chaser_state) }, \
         { "num_sat", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_chaser_status_t, num_sat) }, \
         { "armed", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_chaser_status_t, armed) }, \
         } \
}


/**
 * @brief Pack a chaser_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param control_mode control mode
 * @param chaser_state chaser state
 * @param num_sat GPS number of satellite
 * @param armed armed or disarmed flag
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chaser_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t control_mode, uint8_t chaser_state, uint8_t num_sat, uint8_t armed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_STATUS_LEN];
	_mav_put_uint8_t(buf, 0, control_mode);
	_mav_put_uint8_t(buf, 1, chaser_state);
	_mav_put_uint8_t(buf, 2, num_sat);
	_mav_put_uint8_t(buf, 3, armed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASER_STATUS_LEN);
#else
	mavlink_chaser_status_t packet;
	packet.control_mode = control_mode;
	packet.chaser_state = chaser_state;
	packet.num_sat = num_sat;
	packet.armed = armed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHASER_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CHASER_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHASER_STATUS_LEN, MAVLINK_MSG_ID_CHASER_STATUS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHASER_STATUS_LEN);
#endif
}

/**
 * @brief Pack a chaser_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param control_mode control mode
 * @param chaser_state chaser state
 * @param num_sat GPS number of satellite
 * @param armed armed or disarmed flag
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chaser_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t control_mode,uint8_t chaser_state,uint8_t num_sat,uint8_t armed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_STATUS_LEN];
	_mav_put_uint8_t(buf, 0, control_mode);
	_mav_put_uint8_t(buf, 1, chaser_state);
	_mav_put_uint8_t(buf, 2, num_sat);
	_mav_put_uint8_t(buf, 3, armed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASER_STATUS_LEN);
#else
	mavlink_chaser_status_t packet;
	packet.control_mode = control_mode;
	packet.chaser_state = chaser_state;
	packet.num_sat = num_sat;
	packet.armed = armed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHASER_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CHASER_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHASER_STATUS_LEN, MAVLINK_MSG_ID_CHASER_STATUS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHASER_STATUS_LEN);
#endif
}

/**
 * @brief Encode a chaser_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param chaser_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chaser_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_chaser_status_t* chaser_status)
{
	return mavlink_msg_chaser_status_pack(system_id, component_id, msg, chaser_status->control_mode, chaser_status->chaser_state, chaser_status->num_sat, chaser_status->armed);
}

/**
 * @brief Encode a chaser_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param chaser_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chaser_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_chaser_status_t* chaser_status)
{
	return mavlink_msg_chaser_status_pack_chan(system_id, component_id, chan, msg, chaser_status->control_mode, chaser_status->chaser_state, chaser_status->num_sat, chaser_status->armed);
}

/**
 * @brief Send a chaser_status message
 * @param chan MAVLink channel to send the message
 *
 * @param control_mode control mode
 * @param chaser_state chaser state
 * @param num_sat GPS number of satellite
 * @param armed armed or disarmed flag
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_chaser_status_send(mavlink_channel_t chan, uint8_t control_mode, uint8_t chaser_state, uint8_t num_sat, uint8_t armed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_STATUS_LEN];
	_mav_put_uint8_t(buf, 0, control_mode);
	_mav_put_uint8_t(buf, 1, chaser_state);
	_mav_put_uint8_t(buf, 2, num_sat);
	_mav_put_uint8_t(buf, 3, armed);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_STATUS, buf, MAVLINK_MSG_ID_CHASER_STATUS_LEN, MAVLINK_MSG_ID_CHASER_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_STATUS, buf, MAVLINK_MSG_ID_CHASER_STATUS_LEN);
#endif
#else
	mavlink_chaser_status_t packet;
	packet.control_mode = control_mode;
	packet.chaser_state = chaser_state;
	packet.num_sat = num_sat;
	packet.armed = armed;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_STATUS, (const char *)&packet, MAVLINK_MSG_ID_CHASER_STATUS_LEN, MAVLINK_MSG_ID_CHASER_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_STATUS, (const char *)&packet, MAVLINK_MSG_ID_CHASER_STATUS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_CHASER_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_chaser_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t control_mode, uint8_t chaser_state, uint8_t num_sat, uint8_t armed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, control_mode);
	_mav_put_uint8_t(buf, 1, chaser_state);
	_mav_put_uint8_t(buf, 2, num_sat);
	_mav_put_uint8_t(buf, 3, armed);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_STATUS, buf, MAVLINK_MSG_ID_CHASER_STATUS_LEN, MAVLINK_MSG_ID_CHASER_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_STATUS, buf, MAVLINK_MSG_ID_CHASER_STATUS_LEN);
#endif
#else
	mavlink_chaser_status_t *packet = (mavlink_chaser_status_t *)msgbuf;
	packet->control_mode = control_mode;
	packet->chaser_state = chaser_state;
	packet->num_sat = num_sat;
	packet->armed = armed;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_STATUS, (const char *)packet, MAVLINK_MSG_ID_CHASER_STATUS_LEN, MAVLINK_MSG_ID_CHASER_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_STATUS, (const char *)packet, MAVLINK_MSG_ID_CHASER_STATUS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE CHASER_STATUS UNPACKING


/**
 * @brief Get field control_mode from chaser_status message
 *
 * @return control mode
 */
static inline uint8_t mavlink_msg_chaser_status_get_control_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field chaser_state from chaser_status message
 *
 * @return chaser state
 */
static inline uint8_t mavlink_msg_chaser_status_get_chaser_state(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field num_sat from chaser_status message
 *
 * @return GPS number of satellite
 */
static inline uint8_t mavlink_msg_chaser_status_get_num_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field armed from chaser_status message
 *
 * @return armed or disarmed flag
 */
static inline uint8_t mavlink_msg_chaser_status_get_armed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Decode a chaser_status message into a struct
 *
 * @param msg The message to decode
 * @param chaser_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_chaser_status_decode(const mavlink_message_t* msg, mavlink_chaser_status_t* chaser_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	chaser_status->control_mode = mavlink_msg_chaser_status_get_control_mode(msg);
	chaser_status->chaser_state = mavlink_msg_chaser_status_get_chaser_state(msg);
	chaser_status->num_sat = mavlink_msg_chaser_status_get_num_sat(msg);
	chaser_status->armed = mavlink_msg_chaser_status_get_armed(msg);
#else
	memcpy(chaser_status, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_CHASER_STATUS_LEN);
#endif
}
