// MESSAGE CHASER_CMD PACKING

#define MAVLINK_MSG_ID_CHASER_CMD 200

typedef struct __mavlink_chaser_cmd_t
{
 int32_t p3; ///< 32bit unsigned int value
 int16_t p2; ///< 16bit unsigned int value
 uint8_t command; ///< 1:change state, 2:NOT USE, 3:arm, 4: send millis(debug)
 int8_t p1; ///< 8bit unsigned int value
} mavlink_chaser_cmd_t;

#define MAVLINK_MSG_ID_CHASER_CMD_LEN 8
#define MAVLINK_MSG_ID_200_LEN 8

#define MAVLINK_MSG_ID_CHASER_CMD_CRC 238
#define MAVLINK_MSG_ID_200_CRC 238



#define MAVLINK_MESSAGE_INFO_CHASER_CMD { \
	"CHASER_CMD", \
	4, \
	{  { "p3", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_chaser_cmd_t, p3) }, \
         { "p2", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_chaser_cmd_t, p2) }, \
         { "command", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_chaser_cmd_t, command) }, \
         { "p1", NULL, MAVLINK_TYPE_INT8_T, 0, 7, offsetof(mavlink_chaser_cmd_t, p1) }, \
         } \
}


/**
 * @brief Pack a chaser_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param command 1:change state, 2:NOT USE, 3:arm, 4: send millis(debug)
 * @param p1 8bit unsigned int value
 * @param p2 16bit unsigned int value
 * @param p3 32bit unsigned int value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chaser_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t command, int8_t p1, int16_t p2, int32_t p3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_CMD_LEN];
	_mav_put_int32_t(buf, 0, p3);
	_mav_put_int16_t(buf, 4, p2);
	_mav_put_uint8_t(buf, 6, command);
	_mav_put_int8_t(buf, 7, p1);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASER_CMD_LEN);
#else
	mavlink_chaser_cmd_t packet;
	packet.p3 = p3;
	packet.p2 = p2;
	packet.command = command;
	packet.p1 = p1;

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
 * @param command 1:change state, 2:NOT USE, 3:arm, 4: send millis(debug)
 * @param p1 8bit unsigned int value
 * @param p2 16bit unsigned int value
 * @param p3 32bit unsigned int value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chaser_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t command,int8_t p1,int16_t p2,int32_t p3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_CMD_LEN];
	_mav_put_int32_t(buf, 0, p3);
	_mav_put_int16_t(buf, 4, p2);
	_mav_put_uint8_t(buf, 6, command);
	_mav_put_int8_t(buf, 7, p1);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASER_CMD_LEN);
#else
	mavlink_chaser_cmd_t packet;
	packet.p3 = p3;
	packet.p2 = p2;
	packet.command = command;
	packet.p1 = p1;

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
	return mavlink_msg_chaser_cmd_pack(system_id, component_id, msg, chaser_cmd->command, chaser_cmd->p1, chaser_cmd->p2, chaser_cmd->p3);
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
	return mavlink_msg_chaser_cmd_pack_chan(system_id, component_id, chan, msg, chaser_cmd->command, chaser_cmd->p1, chaser_cmd->p2, chaser_cmd->p3);
}

/**
 * @brief Send a chaser_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param command 1:change state, 2:NOT USE, 3:arm, 4: send millis(debug)
 * @param p1 8bit unsigned int value
 * @param p2 16bit unsigned int value
 * @param p3 32bit unsigned int value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_chaser_cmd_send(mavlink_channel_t chan, uint8_t command, int8_t p1, int16_t p2, int32_t p3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_CMD_LEN];
	_mav_put_int32_t(buf, 0, p3);
	_mav_put_int16_t(buf, 4, p2);
	_mav_put_uint8_t(buf, 6, command);
	_mav_put_int8_t(buf, 7, p1);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_CMD, buf, MAVLINK_MSG_ID_CHASER_CMD_LEN, MAVLINK_MSG_ID_CHASER_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_CMD, buf, MAVLINK_MSG_ID_CHASER_CMD_LEN);
#endif
#else
	mavlink_chaser_cmd_t packet;
	packet.p3 = p3;
	packet.p2 = p2;
	packet.command = command;
	packet.p1 = p1;

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
static inline void mavlink_msg_chaser_cmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t command, int8_t p1, int16_t p2, int32_t p3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int32_t(buf, 0, p3);
	_mav_put_int16_t(buf, 4, p2);
	_mav_put_uint8_t(buf, 6, command);
	_mav_put_int8_t(buf, 7, p1);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_CMD, buf, MAVLINK_MSG_ID_CHASER_CMD_LEN, MAVLINK_MSG_ID_CHASER_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_CMD, buf, MAVLINK_MSG_ID_CHASER_CMD_LEN);
#endif
#else
	mavlink_chaser_cmd_t *packet = (mavlink_chaser_cmd_t *)msgbuf;
	packet->p3 = p3;
	packet->p2 = p2;
	packet->command = command;
	packet->p1 = p1;

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
 * @return 1:change state, 2:NOT USE, 3:arm, 4: send millis(debug)
 */
static inline uint8_t mavlink_msg_chaser_cmd_get_command(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field p1 from chaser_cmd message
 *
 * @return 8bit unsigned int value
 */
static inline int8_t mavlink_msg_chaser_cmd_get_p1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  7);
}

/**
 * @brief Get field p2 from chaser_cmd message
 *
 * @return 16bit unsigned int value
 */
static inline int16_t mavlink_msg_chaser_cmd_get_p2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field p3 from chaser_cmd message
 *
 * @return 32bit unsigned int value
 */
static inline int32_t mavlink_msg_chaser_cmd_get_p3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  0);
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
	chaser_cmd->p3 = mavlink_msg_chaser_cmd_get_p3(msg);
	chaser_cmd->p2 = mavlink_msg_chaser_cmd_get_p2(msg);
	chaser_cmd->command = mavlink_msg_chaser_cmd_get_command(msg);
	chaser_cmd->p1 = mavlink_msg_chaser_cmd_get_p1(msg);
#else
	memcpy(chaser_cmd, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_CHASER_CMD_LEN);
#endif
}
