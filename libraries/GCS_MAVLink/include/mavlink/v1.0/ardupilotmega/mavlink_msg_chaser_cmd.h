// MESSAGE CHASER_CMD PACKING

#define MAVLINK_MSG_ID_CHASER_CMD 200

typedef struct __mavlink_chaser_cmd_t
{
 int8_t command; ///< command for chacer 
 int8_t mode; ///< chaser mode
} mavlink_chaser_cmd_t;

#define MAVLINK_MSG_ID_CHASER_CMD_LEN 2
#define MAVLINK_MSG_ID_200_LEN 2

#define MAVLINK_MSG_ID_CHASER_CMD_CRC 172
#define MAVLINK_MSG_ID_200_CRC 172



#define MAVLINK_MESSAGE_INFO_CHASER_CMD { \
	"CHASER_CMD", \
	2, \
	{  { "command", NULL, MAVLINK_TYPE_INT8_T, 0, 0, offsetof(mavlink_chaser_cmd_t, command) }, \
         { "mode", NULL, MAVLINK_TYPE_INT8_T, 0, 1, offsetof(mavlink_chaser_cmd_t, mode) }, \
         } \
}


/**
 * @brief Pack a chaser_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param command command for chacer 
 * @param mode chaser mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chaser_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int8_t command, int8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_CMD_LEN];
	_mav_put_int8_t(buf, 0, command);
	_mav_put_int8_t(buf, 1, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASER_CMD_LEN);
#else
	mavlink_chaser_cmd_t packet;
	packet.command = command;
	packet.mode = mode;

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
 * @param command command for chacer 
 * @param mode chaser mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chaser_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int8_t command,int8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_CMD_LEN];
	_mav_put_int8_t(buf, 0, command);
	_mav_put_int8_t(buf, 1, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASER_CMD_LEN);
#else
	mavlink_chaser_cmd_t packet;
	packet.command = command;
	packet.mode = mode;

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
	return mavlink_msg_chaser_cmd_pack(system_id, component_id, msg, chaser_cmd->command, chaser_cmd->mode);
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
	return mavlink_msg_chaser_cmd_pack_chan(system_id, component_id, chan, msg, chaser_cmd->command, chaser_cmd->mode);
}

/**
 * @brief Send a chaser_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param command command for chacer 
 * @param mode chaser mode
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_chaser_cmd_send(mavlink_channel_t chan, int8_t command, int8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_CMD_LEN];
	_mav_put_int8_t(buf, 0, command);
	_mav_put_int8_t(buf, 1, mode);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_CMD, buf, MAVLINK_MSG_ID_CHASER_CMD_LEN, MAVLINK_MSG_ID_CHASER_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_CMD, buf, MAVLINK_MSG_ID_CHASER_CMD_LEN);
#endif
#else
	mavlink_chaser_cmd_t packet;
	packet.command = command;
	packet.mode = mode;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_CMD, (const char *)&packet, MAVLINK_MSG_ID_CHASER_CMD_LEN, MAVLINK_MSG_ID_CHASER_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_CMD, (const char *)&packet, MAVLINK_MSG_ID_CHASER_CMD_LEN);
#endif
#endif
}

#endif

// MESSAGE CHASER_CMD UNPACKING


/**
 * @brief Get field command from chaser_cmd message
 *
 * @return command for chacer 
 */
static inline int8_t mavlink_msg_chaser_cmd_get_command(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  0);
}

/**
 * @brief Get field mode from chaser_cmd message
 *
 * @return chaser mode
 */
static inline int8_t mavlink_msg_chaser_cmd_get_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  1);
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
	chaser_cmd->command = mavlink_msg_chaser_cmd_get_command(msg);
	chaser_cmd->mode = mavlink_msg_chaser_cmd_get_mode(msg);
#else
	memcpy(chaser_cmd, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_CHASER_CMD_LEN);
#endif
}
