// MESSAGE CHASER_CMD PACKING

#define MAVLINK_MSG_ID_CHASER_CMD 200

typedef struct __mavlink_chaser_cmd_t
{
 int32_t lat; ///< latitude (degree * 10^7)
 int32_t lon; ///< longitude (degree * 10^7)
 int16_t alt; ///< altitude (cm)
} mavlink_chaser_cmd_t;

#define MAVLINK_MSG_ID_CHASER_CMD_LEN 10
#define MAVLINK_MSG_ID_200_LEN 10

#define MAVLINK_MSG_ID_CHASER_CMD_CRC 85
#define MAVLINK_MSG_ID_200_CRC 85



#define MAVLINK_MESSAGE_INFO_CHASER_CMD { \
	"CHASER_CMD", \
	3, \
	{  { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_chaser_cmd_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_chaser_cmd_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_chaser_cmd_t, alt) }, \
         } \
}


/**
 * @brief Pack a chaser_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat latitude (degree * 10^7)
 * @param lon longitude (degree * 10^7)
 * @param alt altitude (cm)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chaser_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int32_t lat, int32_t lon, int16_t alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_CMD_LEN];
	_mav_put_int32_t(buf, 0, lat);
	_mav_put_int32_t(buf, 4, lon);
	_mav_put_int16_t(buf, 8, alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASER_CMD_LEN);
#else
	mavlink_chaser_cmd_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;

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
 * @param lat latitude (degree * 10^7)
 * @param lon longitude (degree * 10^7)
 * @param alt altitude (cm)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chaser_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int32_t lat,int32_t lon,int16_t alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_CMD_LEN];
	_mav_put_int32_t(buf, 0, lat);
	_mav_put_int32_t(buf, 4, lon);
	_mav_put_int16_t(buf, 8, alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASER_CMD_LEN);
#else
	mavlink_chaser_cmd_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;

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
	return mavlink_msg_chaser_cmd_pack(system_id, component_id, msg, chaser_cmd->lat, chaser_cmd->lon, chaser_cmd->alt);
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
	return mavlink_msg_chaser_cmd_pack_chan(system_id, component_id, chan, msg, chaser_cmd->lat, chaser_cmd->lon, chaser_cmd->alt);
}

/**
 * @brief Send a chaser_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param lat latitude (degree * 10^7)
 * @param lon longitude (degree * 10^7)
 * @param alt altitude (cm)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_chaser_cmd_send(mavlink_channel_t chan, int32_t lat, int32_t lon, int16_t alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHASER_CMD_LEN];
	_mav_put_int32_t(buf, 0, lat);
	_mav_put_int32_t(buf, 4, lon);
	_mav_put_int16_t(buf, 8, alt);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_CMD, buf, MAVLINK_MSG_ID_CHASER_CMD_LEN, MAVLINK_MSG_ID_CHASER_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASER_CMD, buf, MAVLINK_MSG_ID_CHASER_CMD_LEN);
#endif
#else
	mavlink_chaser_cmd_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;

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
 * @brief Get field lat from chaser_cmd message
 *
 * @return latitude (degree * 10^7)
 */
static inline int32_t mavlink_msg_chaser_cmd_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field lon from chaser_cmd message
 *
 * @return longitude (degree * 10^7)
 */
static inline int32_t mavlink_msg_chaser_cmd_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field alt from chaser_cmd message
 *
 * @return altitude (cm)
 */
static inline int16_t mavlink_msg_chaser_cmd_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  8);
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
	chaser_cmd->lat = mavlink_msg_chaser_cmd_get_lat(msg);
	chaser_cmd->lon = mavlink_msg_chaser_cmd_get_lon(msg);
	chaser_cmd->alt = mavlink_msg_chaser_cmd_get_alt(msg);
#else
	memcpy(chaser_cmd, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_CHASER_CMD_LEN);
#endif
}
