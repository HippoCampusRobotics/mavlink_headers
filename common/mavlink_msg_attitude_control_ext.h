#pragma once
// MESSAGE ATTITUDE_CONTROL_EXT PACKING

#define MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT 229


typedef struct __mavlink_attitude_control_ext_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.*/
 float thrust; /*<  Thrust that is fedthrough to the mixer*/
 float roll; /*<  */
 float pitch; /*<  */
 float yaw; /*<  */
} mavlink_attitude_control_ext_t;

#define MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_LEN 24
#define MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_MIN_LEN 24
#define MAVLINK_MSG_ID_229_LEN 24
#define MAVLINK_MSG_ID_229_MIN_LEN 24

#define MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_CRC 123
#define MAVLINK_MSG_ID_229_CRC 123



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ATTITUDE_CONTROL_EXT { \
    229, \
    "ATTITUDE_CONTROL_EXT", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_attitude_control_ext_t, time_usec) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_attitude_control_ext_t, thrust) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_attitude_control_ext_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_attitude_control_ext_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_attitude_control_ext_t, yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ATTITUDE_CONTROL_EXT { \
    "ATTITUDE_CONTROL_EXT", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_attitude_control_ext_t, time_usec) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_attitude_control_ext_t, thrust) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_attitude_control_ext_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_attitude_control_ext_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_attitude_control_ext_t, yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a attitude_control_ext message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 * @param thrust  Thrust that is fedthrough to the mixer
 * @param roll  
 * @param pitch  
 * @param yaw  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_control_ext_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float thrust, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, thrust);
    _mav_put_float(buf, 12, roll);
    _mav_put_float(buf, 16, pitch);
    _mav_put_float(buf, 20, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_LEN);
#else
    mavlink_attitude_control_ext_t packet;
    packet.time_usec = time_usec;
    packet.thrust = thrust;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_LEN, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_CRC);
}

/**
 * @brief Pack a attitude_control_ext message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 * @param thrust  Thrust that is fedthrough to the mixer
 * @param roll  
 * @param pitch  
 * @param yaw  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_control_ext_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float thrust,float roll,float pitch,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, thrust);
    _mav_put_float(buf, 12, roll);
    _mav_put_float(buf, 16, pitch);
    _mav_put_float(buf, 20, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_LEN);
#else
    mavlink_attitude_control_ext_t packet;
    packet.time_usec = time_usec;
    packet.thrust = thrust;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_LEN, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_CRC);
}

/**
 * @brief Encode a attitude_control_ext struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param attitude_control_ext C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_control_ext_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_attitude_control_ext_t* attitude_control_ext)
{
    return mavlink_msg_attitude_control_ext_pack(system_id, component_id, msg, attitude_control_ext->time_usec, attitude_control_ext->thrust, attitude_control_ext->roll, attitude_control_ext->pitch, attitude_control_ext->yaw);
}

/**
 * @brief Encode a attitude_control_ext struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param attitude_control_ext C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_control_ext_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_attitude_control_ext_t* attitude_control_ext)
{
    return mavlink_msg_attitude_control_ext_pack_chan(system_id, component_id, chan, msg, attitude_control_ext->time_usec, attitude_control_ext->thrust, attitude_control_ext->roll, attitude_control_ext->pitch, attitude_control_ext->yaw);
}

/**
 * @brief Send a attitude_control_ext message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 * @param thrust  Thrust that is fedthrough to the mixer
 * @param roll  
 * @param pitch  
 * @param yaw  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_attitude_control_ext_send(mavlink_channel_t chan, uint64_t time_usec, float thrust, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, thrust);
    _mav_put_float(buf, 12, roll);
    _mav_put_float(buf, 16, pitch);
    _mav_put_float(buf, 20, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT, buf, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_LEN, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_CRC);
#else
    mavlink_attitude_control_ext_t packet;
    packet.time_usec = time_usec;
    packet.thrust = thrust;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT, (const char *)&packet, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_LEN, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_CRC);
#endif
}

/**
 * @brief Send a attitude_control_ext message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_attitude_control_ext_send_struct(mavlink_channel_t chan, const mavlink_attitude_control_ext_t* attitude_control_ext)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_attitude_control_ext_send(chan, attitude_control_ext->time_usec, attitude_control_ext->thrust, attitude_control_ext->roll, attitude_control_ext->pitch, attitude_control_ext->yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT, (const char *)attitude_control_ext, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_LEN, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_CRC);
#endif
}

#if MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_attitude_control_ext_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float thrust, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, thrust);
    _mav_put_float(buf, 12, roll);
    _mav_put_float(buf, 16, pitch);
    _mav_put_float(buf, 20, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT, buf, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_LEN, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_CRC);
#else
    mavlink_attitude_control_ext_t *packet = (mavlink_attitude_control_ext_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->thrust = thrust;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT, (const char *)packet, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_LEN, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_CRC);
#endif
}
#endif

#endif

// MESSAGE ATTITUDE_CONTROL_EXT UNPACKING


/**
 * @brief Get field time_usec from attitude_control_ext message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 */
static inline uint64_t mavlink_msg_attitude_control_ext_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field thrust from attitude_control_ext message
 *
 * @return  Thrust that is fedthrough to the mixer
 */
static inline float mavlink_msg_attitude_control_ext_get_thrust(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field roll from attitude_control_ext message
 *
 * @return  
 */
static inline float mavlink_msg_attitude_control_ext_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pitch from attitude_control_ext message
 *
 * @return  
 */
static inline float mavlink_msg_attitude_control_ext_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field yaw from attitude_control_ext message
 *
 * @return  
 */
static inline float mavlink_msg_attitude_control_ext_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a attitude_control_ext message into a struct
 *
 * @param msg The message to decode
 * @param attitude_control_ext C-struct to decode the message contents into
 */
static inline void mavlink_msg_attitude_control_ext_decode(const mavlink_message_t* msg, mavlink_attitude_control_ext_t* attitude_control_ext)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    attitude_control_ext->time_usec = mavlink_msg_attitude_control_ext_get_time_usec(msg);
    attitude_control_ext->thrust = mavlink_msg_attitude_control_ext_get_thrust(msg);
    attitude_control_ext->roll = mavlink_msg_attitude_control_ext_get_roll(msg);
    attitude_control_ext->pitch = mavlink_msg_attitude_control_ext_get_pitch(msg);
    attitude_control_ext->yaw = mavlink_msg_attitude_control_ext_get_yaw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_LEN? msg->len : MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_LEN;
        memset(attitude_control_ext, 0, MAVLINK_MSG_ID_ATTITUDE_CONTROL_EXT_LEN);
    memcpy(attitude_control_ext, _MAV_PAYLOAD(msg), len);
#endif
}
