#pragma once
// MESSAGE MIXER_FEEDTHROUGH PACKING

#define MAVLINK_MSG_ID_MIXER_FEEDTHROUGH 228


typedef struct __mavlink_mixer_feedthrough_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.*/
 float motor_ul; /*<  */
 float motor_ur; /*<  */
 float motor_ll; /*<  */
 float motor_lr; /*<  */
} mavlink_mixer_feedthrough_t;

#define MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_LEN 24
#define MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_MIN_LEN 24
#define MAVLINK_MSG_ID_228_LEN 24
#define MAVLINK_MSG_ID_228_MIN_LEN 24

#define MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_CRC 148
#define MAVLINK_MSG_ID_228_CRC 148



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MIXER_FEEDTHROUGH { \
    228, \
    "MIXER_FEEDTHROUGH", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mixer_feedthrough_t, time_usec) }, \
         { "motor_ul", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mixer_feedthrough_t, motor_ul) }, \
         { "motor_ur", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mixer_feedthrough_t, motor_ur) }, \
         { "motor_ll", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_mixer_feedthrough_t, motor_ll) }, \
         { "motor_lr", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_mixer_feedthrough_t, motor_lr) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MIXER_FEEDTHROUGH { \
    "MIXER_FEEDTHROUGH", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mixer_feedthrough_t, time_usec) }, \
         { "motor_ul", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mixer_feedthrough_t, motor_ul) }, \
         { "motor_ur", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mixer_feedthrough_t, motor_ur) }, \
         { "motor_ll", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_mixer_feedthrough_t, motor_ll) }, \
         { "motor_lr", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_mixer_feedthrough_t, motor_lr) }, \
         } \
}
#endif

/**
 * @brief Pack a mixer_feedthrough message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 * @param motor_ul  
 * @param motor_ur  
 * @param motor_ll  
 * @param motor_lr  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mixer_feedthrough_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float motor_ul, float motor_ur, float motor_ll, float motor_lr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, motor_ul);
    _mav_put_float(buf, 12, motor_ur);
    _mav_put_float(buf, 16, motor_ll);
    _mav_put_float(buf, 20, motor_lr);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_LEN);
#else
    mavlink_mixer_feedthrough_t packet;
    packet.time_usec = time_usec;
    packet.motor_ul = motor_ul;
    packet.motor_ur = motor_ur;
    packet.motor_ll = motor_ll;
    packet.motor_lr = motor_lr;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MIXER_FEEDTHROUGH;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_MIN_LEN, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_LEN, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_CRC);
}

/**
 * @brief Pack a mixer_feedthrough message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 * @param motor_ul  
 * @param motor_ur  
 * @param motor_ll  
 * @param motor_lr  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mixer_feedthrough_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float motor_ul,float motor_ur,float motor_ll,float motor_lr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, motor_ul);
    _mav_put_float(buf, 12, motor_ur);
    _mav_put_float(buf, 16, motor_ll);
    _mav_put_float(buf, 20, motor_lr);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_LEN);
#else
    mavlink_mixer_feedthrough_t packet;
    packet.time_usec = time_usec;
    packet.motor_ul = motor_ul;
    packet.motor_ur = motor_ur;
    packet.motor_ll = motor_ll;
    packet.motor_lr = motor_lr;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MIXER_FEEDTHROUGH;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_MIN_LEN, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_LEN, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_CRC);
}

/**
 * @brief Encode a mixer_feedthrough struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mixer_feedthrough C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mixer_feedthrough_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mixer_feedthrough_t* mixer_feedthrough)
{
    return mavlink_msg_mixer_feedthrough_pack(system_id, component_id, msg, mixer_feedthrough->time_usec, mixer_feedthrough->motor_ul, mixer_feedthrough->motor_ur, mixer_feedthrough->motor_ll, mixer_feedthrough->motor_lr);
}

/**
 * @brief Encode a mixer_feedthrough struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mixer_feedthrough C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mixer_feedthrough_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mixer_feedthrough_t* mixer_feedthrough)
{
    return mavlink_msg_mixer_feedthrough_pack_chan(system_id, component_id, chan, msg, mixer_feedthrough->time_usec, mixer_feedthrough->motor_ul, mixer_feedthrough->motor_ur, mixer_feedthrough->motor_ll, mixer_feedthrough->motor_lr);
}

/**
 * @brief Send a mixer_feedthrough message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 * @param motor_ul  
 * @param motor_ur  
 * @param motor_ll  
 * @param motor_lr  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mixer_feedthrough_send(mavlink_channel_t chan, uint64_t time_usec, float motor_ul, float motor_ur, float motor_ll, float motor_lr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, motor_ul);
    _mav_put_float(buf, 12, motor_ur);
    _mav_put_float(buf, 16, motor_ll);
    _mav_put_float(buf, 20, motor_lr);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH, buf, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_MIN_LEN, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_LEN, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_CRC);
#else
    mavlink_mixer_feedthrough_t packet;
    packet.time_usec = time_usec;
    packet.motor_ul = motor_ul;
    packet.motor_ur = motor_ur;
    packet.motor_ll = motor_ll;
    packet.motor_lr = motor_lr;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH, (const char *)&packet, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_MIN_LEN, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_LEN, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_CRC);
#endif
}

/**
 * @brief Send a mixer_feedthrough message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mixer_feedthrough_send_struct(mavlink_channel_t chan, const mavlink_mixer_feedthrough_t* mixer_feedthrough)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mixer_feedthrough_send(chan, mixer_feedthrough->time_usec, mixer_feedthrough->motor_ul, mixer_feedthrough->motor_ur, mixer_feedthrough->motor_ll, mixer_feedthrough->motor_lr);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH, (const char *)mixer_feedthrough, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_MIN_LEN, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_LEN, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_CRC);
#endif
}

#if MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mixer_feedthrough_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float motor_ul, float motor_ur, float motor_ll, float motor_lr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, motor_ul);
    _mav_put_float(buf, 12, motor_ur);
    _mav_put_float(buf, 16, motor_ll);
    _mav_put_float(buf, 20, motor_lr);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH, buf, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_MIN_LEN, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_LEN, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_CRC);
#else
    mavlink_mixer_feedthrough_t *packet = (mavlink_mixer_feedthrough_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->motor_ul = motor_ul;
    packet->motor_ur = motor_ur;
    packet->motor_ll = motor_ll;
    packet->motor_lr = motor_lr;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH, (const char *)packet, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_MIN_LEN, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_LEN, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_CRC);
#endif
}
#endif

#endif

// MESSAGE MIXER_FEEDTHROUGH UNPACKING


/**
 * @brief Get field time_usec from mixer_feedthrough message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 */
static inline uint64_t mavlink_msg_mixer_feedthrough_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field motor_ul from mixer_feedthrough message
 *
 * @return  
 */
static inline float mavlink_msg_mixer_feedthrough_get_motor_ul(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field motor_ur from mixer_feedthrough message
 *
 * @return  
 */
static inline float mavlink_msg_mixer_feedthrough_get_motor_ur(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field motor_ll from mixer_feedthrough message
 *
 * @return  
 */
static inline float mavlink_msg_mixer_feedthrough_get_motor_ll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field motor_lr from mixer_feedthrough message
 *
 * @return  
 */
static inline float mavlink_msg_mixer_feedthrough_get_motor_lr(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a mixer_feedthrough message into a struct
 *
 * @param msg The message to decode
 * @param mixer_feedthrough C-struct to decode the message contents into
 */
static inline void mavlink_msg_mixer_feedthrough_decode(const mavlink_message_t* msg, mavlink_mixer_feedthrough_t* mixer_feedthrough)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mixer_feedthrough->time_usec = mavlink_msg_mixer_feedthrough_get_time_usec(msg);
    mixer_feedthrough->motor_ul = mavlink_msg_mixer_feedthrough_get_motor_ul(msg);
    mixer_feedthrough->motor_ur = mavlink_msg_mixer_feedthrough_get_motor_ur(msg);
    mixer_feedthrough->motor_ll = mavlink_msg_mixer_feedthrough_get_motor_ll(msg);
    mixer_feedthrough->motor_lr = mavlink_msg_mixer_feedthrough_get_motor_lr(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_LEN? msg->len : MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_LEN;
        memset(mixer_feedthrough, 0, MAVLINK_MSG_ID_MIXER_FEEDTHROUGH_LEN);
    memcpy(mixer_feedthrough, _MAV_PAYLOAD(msg), len);
#endif
}
