// MESSAGE CHASER_CMD PACKING

#define MAVLINK_MSG_ID_CHASER_CMD 200

typedef struct __mavlink_chaser_cmd_t
{
 uint16_t throttle; ///< throttle(0-1000)
 uint8_t command; ///< 0:do nothing, 1:change state, 2:change throttle, 3:arm
 uint8_t state; ///< chaser state
} mavlink_chaser_cmd_t;

#define MAVLINK_MSG_ID_CHASER_CMD_LEN 4
#define MAVLINK_MSG_ID_200_LEN 4

#define MAVLINK_MSG_ID_CHASER_CMD_CRC 36
#define MAVLINK_MSG_ID_200_CRC 36



#define MAVLINK_MESSAGE_INFO_CHASER_CMD { \
	"CHASER_CMD", \
	3, \
	{  { "throttle", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_chaser_cmd_t, throttle) }, \
         { "command", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_chaser_cmd_t, command) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_chaser_cmd_t, state) }, \
         } \
}


/**
 * @brief Pack a chaser_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param command 0:do nothing, 1:change state, 2:change throttle, 3:arm
 * @param state chaser state
 * @param throttle throttle(0-1000)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chaser_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t command, uint8_t state, uint16_t throttle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_CMD_LEN];
	_mav_put_uint16_t(buf, 0, throttle);
	_mav_put_uint8_t(buf, 2, command);
	_mav_put_uint8_t(buf, 3, state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASER_CMD_LEN);
#else
	mavlink_chaser_cmd_t packet;
	packet.throttle = throttle;
	packet.command = command;
	packet.state = state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHASER_CMD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CHASER_CMD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHASER_CMD_LEN, MAVLINK_MSG_ID_CHASER_CMD_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHASER_CMD_LEN);
#endif
}

/**
 * @brief Pack a chaser_cmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param command 0:do nothing, 1:change state, 2:change throttle, 3:arm
 * @param state chaser state
 * @param throttle throttle(0-1000)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chaser_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t command,uint8_t state,uint16_t throttle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_CMD_LEN];
	_mav_put_uint16_t(buf, 0, throttle);
	_mav_put_uint8_t(buf, 2, command);
	_mav_put_uint8_t(buf, 3, state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASER_CMD_LEN);
#else
	mavlink_chaser_cmd_t packet;
	packet.throttle = throttle;
	packet.command = command;
	packet.state = state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHASER_CMD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CHASER_CMD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHASER_CMD_LEN, MAVLINK_MSG_ID_CHASER_CMD_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHASER_CMD_LEN);
#endif
}

/**
 * @brief Encode a chaser_cmd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param chaser_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chaser_cmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_chaser_cmd_t* chaser_cmd)
{
	return mavlink_msg_chaser_cmd_pack(system_id, component_id, msg, chaser_cmd->command, chaser_cmd->state, chaser_cmd->throttle);
}

/**
 * @brief Encode a chaser_cmd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param chaser_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chaser_cmd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_chaser_cmd_t* chaser_cmd)
{
	return mavlink_msg_chaser_cmd_pack_chan(system_id, component_id, chan, msg, chaser_cmd->command, chaser_cmd->state, chaser_cmd->throttle);
}

/**
 * @brief Send a chaser_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param command 0:do nothing, 1:change state, 2:change throttle, 3:arm
 * @param state chaser state
 * @param throttle throttle(0-1000)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_chaser_cmd_send(mavlink_channel_t chan, uint8_t command, uint8_t state, uint16_t throttle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_CMD_LEN];
	_mav_put_uint16_t(buf, 0, throttle);
	_mav_put_uint8_t(buf, 2, command);
	_mav_put_uint8_t(buf, 3, state);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_CMD, buf, MAVLINK_MSG_ID_CHASER_CMD_LEN, MAVLINK_MSG_ID_CHASER_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_CMD, buf, MAVLINK_MSG_ID_CHASER_CMD_LEN);
#endif
#else
	mavlink_chaser_cmd_t packet;
	packet.throttle = throttle;
	packet.command = command;
	packet.state = state;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_CMD, (const char *)&packet, MAVLINK_MSG_ID_CHASER_CMD_LEN, MAVLINK_MSG_ID_CHASER_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_CMD, (const char *)&packet, MAVLINK_MSG_ID_CHASER_CMD_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_CHASER_CMD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_chaser_cmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t command, uint8_t state, uint16_t throttle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, throttle);
	_mav_put_uint8_t(buf, 2, command);
	_mav_put_uint8_t(buf, 3, state);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_CMD, buf, MAVLINK_MSG_ID_CHASER_CMD_LEN, MAVLINK_MSG_ID_CHASER_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_CMD, buf, MAVLINK_MSG_ID_CHASER_CMD_LEN);
#endif
#else
	mavlink_chaser_cmd_t *packet = (mavlink_chaser_cmd_t *)msgbuf;
	packet->throttle = throttle;
	packet->command = command;
	packet->state = state;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_CMD, (const char *)packet, MAVLINK_MSG_ID_CHASER_CMD_LEN, MAVLINK_MSG_ID_CHASER_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_CMD, (const char *)packet, MAVLINK_MSG_ID_CHASER_CMD_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE CHASER_CMD UNPACKING


/**
 * @brief Get field command from chaser_cmd message
 *
 * @return 0:do nothing, 1:change state, 2:change throttle, 3:arm
 */
static inline uint8_t mavlink_msg_chaser_cmd_get_command(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field state from chaser_cmd message
 *
 * @return chaser state
 */
static inline uint8_t mavlink_msg_chaser_cmd_get_state(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field throttle from chaser_cmd message
 *
 * @return throttle(0-1000)
 */
static inline uint16_t mavlink_msg_chaser_cmd_get_throttle(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a chaser_cmd message into a struct
 *
 * @param msg The message to decode
 * @param chaser_cmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_chaser_cmd_decode(const mavlink_message_t* msg, mavlink_chaser_cmd_t* chaser_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP
	chaser_cmd->throttle = mavlink_msg_chaser_cmd_get_throttle(msg);
	chaser_cmd->command = mavlink_msg_chaser_cmd_get_command(msg);
	chaser_cmd->state = mavlink_msg_chaser_cmd_get_state(msg);
#else
	memcpy(chaser_cmd, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_CHASER_CMD_LEN);
#endif
}
