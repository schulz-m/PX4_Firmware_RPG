// MESSAGE EMERGENCY_EKF PACKING

#define MAVLINK_MSG_ID_EMERGENCY_EKF 13

typedef struct __mavlink_emergency_ekf_t
{
 uint64_t time_usec; ///<  Time stamp [microseconds].
 float h_W; ///< Height in World Coordinates [m].
 float u_B; ///< X-Speed in Body Coordinates [m/s].
 float v_B; ///< Y-Speed in Body Coordinates [m/s].
 float w_B; ///< Z-Speed in Body Coordinates [m/s].
 float q1; ///< Attitude quaternion component 1 [-].
 float q2; ///< Attitude quaternion component 2 [-].
 float q3; ///< Attitude quaternion component 3 [-].
 float q4; ///< Attitude quaternion component 4 [-].
 float roll; ///< 	Attitude roll expressed as Euler angles, not recommended except for human-readable outputs [rad].
 float pitch; ///< 	Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs [rad].
 float yaw; ///< 	Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs [rad].
 float p_0; ///< Reference pressure state [mbar/hPa].
 float h_0; ///< Reference height for pressure readings - constant[m].
 float b_s; ///< Calibration variable - ground bias[m].
} mavlink_emergency_ekf_t;

#define MAVLINK_MSG_ID_EMERGENCY_EKF_LEN 64
#define MAVLINK_MSG_ID_13_LEN 64

#define MAVLINK_MSG_ID_EMERGENCY_EKF_CRC 14
#define MAVLINK_MSG_ID_13_CRC 14



#define MAVLINK_MESSAGE_INFO_EMERGENCY_EKF { \
	"EMERGENCY_EKF", \
	15, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_emergency_ekf_t, time_usec) }, \
         { "h_W", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_emergency_ekf_t, h_W) }, \
         { "u_B", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_emergency_ekf_t, u_B) }, \
         { "v_B", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_emergency_ekf_t, v_B) }, \
         { "w_B", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_emergency_ekf_t, w_B) }, \
         { "q1", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_emergency_ekf_t, q1) }, \
         { "q2", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_emergency_ekf_t, q2) }, \
         { "q3", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_emergency_ekf_t, q3) }, \
         { "q4", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_emergency_ekf_t, q4) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_emergency_ekf_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_emergency_ekf_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_emergency_ekf_t, yaw) }, \
         { "p_0", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_emergency_ekf_t, p_0) }, \
         { "h_0", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_emergency_ekf_t, h_0) }, \
         { "b_s", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_emergency_ekf_t, b_s) }, \
         } \
}


/**
 * @brief Pack a emergency_ekf message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec  Time stamp [microseconds].
 * @param h_W Height in World Coordinates [m].
 * @param u_B X-Speed in Body Coordinates [m/s].
 * @param v_B Y-Speed in Body Coordinates [m/s].
 * @param w_B Z-Speed in Body Coordinates [m/s].
 * @param q1 Attitude quaternion component 1 [-].
 * @param q2 Attitude quaternion component 2 [-].
 * @param q3 Attitude quaternion component 3 [-].
 * @param q4 Attitude quaternion component 4 [-].
 * @param roll 	Attitude roll expressed as Euler angles, not recommended except for human-readable outputs [rad].
 * @param pitch 	Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs [rad].
 * @param yaw 	Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs [rad].
 * @param p_0 Reference pressure state [mbar/hPa].
 * @param h_0 Reference height for pressure readings - constant[m].
 * @param b_s Calibration variable - ground bias[m].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_emergency_ekf_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, float h_W, float u_B, float v_B, float w_B, float q1, float q2, float q3, float q4, float roll, float pitch, float yaw, float p_0, float h_0, float b_s)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EMERGENCY_EKF_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, h_W);
	_mav_put_float(buf, 12, u_B);
	_mav_put_float(buf, 16, v_B);
	_mav_put_float(buf, 20, w_B);
	_mav_put_float(buf, 24, q1);
	_mav_put_float(buf, 28, q2);
	_mav_put_float(buf, 32, q3);
	_mav_put_float(buf, 36, q4);
	_mav_put_float(buf, 40, roll);
	_mav_put_float(buf, 44, pitch);
	_mav_put_float(buf, 48, yaw);
	_mav_put_float(buf, 52, p_0);
	_mav_put_float(buf, 56, h_0);
	_mav_put_float(buf, 60, b_s);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN);
#else
	mavlink_emergency_ekf_t packet;
	packet.time_usec = time_usec;
	packet.h_W = h_W;
	packet.u_B = u_B;
	packet.v_B = v_B;
	packet.w_B = w_B;
	packet.q1 = q1;
	packet.q2 = q2;
	packet.q3 = q3;
	packet.q4 = q4;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.p_0 = p_0;
	packet.h_0 = h_0;
	packet.b_s = b_s;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_EMERGENCY_EKF;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN, MAVLINK_MSG_ID_EMERGENCY_EKF_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN);
#endif
}

/**
 * @brief Pack a emergency_ekf message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec  Time stamp [microseconds].
 * @param h_W Height in World Coordinates [m].
 * @param u_B X-Speed in Body Coordinates [m/s].
 * @param v_B Y-Speed in Body Coordinates [m/s].
 * @param w_B Z-Speed in Body Coordinates [m/s].
 * @param q1 Attitude quaternion component 1 [-].
 * @param q2 Attitude quaternion component 2 [-].
 * @param q3 Attitude quaternion component 3 [-].
 * @param q4 Attitude quaternion component 4 [-].
 * @param roll 	Attitude roll expressed as Euler angles, not recommended except for human-readable outputs [rad].
 * @param pitch 	Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs [rad].
 * @param yaw 	Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs [rad].
 * @param p_0 Reference pressure state [mbar/hPa].
 * @param h_0 Reference height for pressure readings - constant[m].
 * @param b_s Calibration variable - ground bias[m].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_emergency_ekf_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,float h_W,float u_B,float v_B,float w_B,float q1,float q2,float q3,float q4,float roll,float pitch,float yaw,float p_0,float h_0,float b_s)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EMERGENCY_EKF_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, h_W);
	_mav_put_float(buf, 12, u_B);
	_mav_put_float(buf, 16, v_B);
	_mav_put_float(buf, 20, w_B);
	_mav_put_float(buf, 24, q1);
	_mav_put_float(buf, 28, q2);
	_mav_put_float(buf, 32, q3);
	_mav_put_float(buf, 36, q4);
	_mav_put_float(buf, 40, roll);
	_mav_put_float(buf, 44, pitch);
	_mav_put_float(buf, 48, yaw);
	_mav_put_float(buf, 52, p_0);
	_mav_put_float(buf, 56, h_0);
	_mav_put_float(buf, 60, b_s);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN);
#else
	mavlink_emergency_ekf_t packet;
	packet.time_usec = time_usec;
	packet.h_W = h_W;
	packet.u_B = u_B;
	packet.v_B = v_B;
	packet.w_B = w_B;
	packet.q1 = q1;
	packet.q2 = q2;
	packet.q3 = q3;
	packet.q4 = q4;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.p_0 = p_0;
	packet.h_0 = h_0;
	packet.b_s = b_s;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_EMERGENCY_EKF;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN, MAVLINK_MSG_ID_EMERGENCY_EKF_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN);
#endif
}

/**
 * @brief Encode a emergency_ekf struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param emergency_ekf C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_emergency_ekf_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_emergency_ekf_t* emergency_ekf)
{
	return mavlink_msg_emergency_ekf_pack(system_id, component_id, msg, emergency_ekf->time_usec, emergency_ekf->h_W, emergency_ekf->u_B, emergency_ekf->v_B, emergency_ekf->w_B, emergency_ekf->q1, emergency_ekf->q2, emergency_ekf->q3, emergency_ekf->q4, emergency_ekf->roll, emergency_ekf->pitch, emergency_ekf->yaw, emergency_ekf->p_0, emergency_ekf->h_0, emergency_ekf->b_s);
}

/**
 * @brief Encode a emergency_ekf struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param emergency_ekf C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_emergency_ekf_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_emergency_ekf_t* emergency_ekf)
{
	return mavlink_msg_emergency_ekf_pack_chan(system_id, component_id, chan, msg, emergency_ekf->time_usec, emergency_ekf->h_W, emergency_ekf->u_B, emergency_ekf->v_B, emergency_ekf->w_B, emergency_ekf->q1, emergency_ekf->q2, emergency_ekf->q3, emergency_ekf->q4, emergency_ekf->roll, emergency_ekf->pitch, emergency_ekf->yaw, emergency_ekf->p_0, emergency_ekf->h_0, emergency_ekf->b_s);
}

/**
 * @brief Send a emergency_ekf message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec  Time stamp [microseconds].
 * @param h_W Height in World Coordinates [m].
 * @param u_B X-Speed in Body Coordinates [m/s].
 * @param v_B Y-Speed in Body Coordinates [m/s].
 * @param w_B Z-Speed in Body Coordinates [m/s].
 * @param q1 Attitude quaternion component 1 [-].
 * @param q2 Attitude quaternion component 2 [-].
 * @param q3 Attitude quaternion component 3 [-].
 * @param q4 Attitude quaternion component 4 [-].
 * @param roll 	Attitude roll expressed as Euler angles, not recommended except for human-readable outputs [rad].
 * @param pitch 	Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs [rad].
 * @param yaw 	Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs [rad].
 * @param p_0 Reference pressure state [mbar/hPa].
 * @param h_0 Reference height for pressure readings - constant[m].
 * @param b_s Calibration variable - ground bias[m].
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_emergency_ekf_send(mavlink_channel_t chan, uint64_t time_usec, float h_W, float u_B, float v_B, float w_B, float q1, float q2, float q3, float q4, float roll, float pitch, float yaw, float p_0, float h_0, float b_s)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EMERGENCY_EKF_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, h_W);
	_mav_put_float(buf, 12, u_B);
	_mav_put_float(buf, 16, v_B);
	_mav_put_float(buf, 20, w_B);
	_mav_put_float(buf, 24, q1);
	_mav_put_float(buf, 28, q2);
	_mav_put_float(buf, 32, q3);
	_mav_put_float(buf, 36, q4);
	_mav_put_float(buf, 40, roll);
	_mav_put_float(buf, 44, pitch);
	_mav_put_float(buf, 48, yaw);
	_mav_put_float(buf, 52, p_0);
	_mav_put_float(buf, 56, h_0);
	_mav_put_float(buf, 60, b_s);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EMERGENCY_EKF, buf, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN, MAVLINK_MSG_ID_EMERGENCY_EKF_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EMERGENCY_EKF, buf, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN);
#endif
#else
	mavlink_emergency_ekf_t packet;
	packet.time_usec = time_usec;
	packet.h_W = h_W;
	packet.u_B = u_B;
	packet.v_B = v_B;
	packet.w_B = w_B;
	packet.q1 = q1;
	packet.q2 = q2;
	packet.q3 = q3;
	packet.q4 = q4;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.p_0 = p_0;
	packet.h_0 = h_0;
	packet.b_s = b_s;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EMERGENCY_EKF, (const char *)&packet, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN, MAVLINK_MSG_ID_EMERGENCY_EKF_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EMERGENCY_EKF, (const char *)&packet, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_EMERGENCY_EKF_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_emergency_ekf_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float h_W, float u_B, float v_B, float w_B, float q1, float q2, float q3, float q4, float roll, float pitch, float yaw, float p_0, float h_0, float b_s)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, h_W);
	_mav_put_float(buf, 12, u_B);
	_mav_put_float(buf, 16, v_B);
	_mav_put_float(buf, 20, w_B);
	_mav_put_float(buf, 24, q1);
	_mav_put_float(buf, 28, q2);
	_mav_put_float(buf, 32, q3);
	_mav_put_float(buf, 36, q4);
	_mav_put_float(buf, 40, roll);
	_mav_put_float(buf, 44, pitch);
	_mav_put_float(buf, 48, yaw);
	_mav_put_float(buf, 52, p_0);
	_mav_put_float(buf, 56, h_0);
	_mav_put_float(buf, 60, b_s);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EMERGENCY_EKF, buf, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN, MAVLINK_MSG_ID_EMERGENCY_EKF_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EMERGENCY_EKF, buf, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN);
#endif
#else
	mavlink_emergency_ekf_t *packet = (mavlink_emergency_ekf_t *)msgbuf;
	packet->time_usec = time_usec;
	packet->h_W = h_W;
	packet->u_B = u_B;
	packet->v_B = v_B;
	packet->w_B = w_B;
	packet->q1 = q1;
	packet->q2 = q2;
	packet->q3 = q3;
	packet->q4 = q4;
	packet->roll = roll;
	packet->pitch = pitch;
	packet->yaw = yaw;
	packet->p_0 = p_0;
	packet->h_0 = h_0;
	packet->b_s = b_s;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EMERGENCY_EKF, (const char *)packet, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN, MAVLINK_MSG_ID_EMERGENCY_EKF_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EMERGENCY_EKF, (const char *)packet, MAVLINK_MSG_ID_EMERGENCY_EKF_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE EMERGENCY_EKF UNPACKING


/**
 * @brief Get field time_usec from emergency_ekf message
 *
 * @return  Time stamp [microseconds].
 */
static inline uint64_t mavlink_msg_emergency_ekf_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field h_W from emergency_ekf message
 *
 * @return Height in World Coordinates [m].
 */
static inline float mavlink_msg_emergency_ekf_get_h_W(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field u_B from emergency_ekf message
 *
 * @return X-Speed in Body Coordinates [m/s].
 */
static inline float mavlink_msg_emergency_ekf_get_u_B(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field v_B from emergency_ekf message
 *
 * @return Y-Speed in Body Coordinates [m/s].
 */
static inline float mavlink_msg_emergency_ekf_get_v_B(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field w_B from emergency_ekf message
 *
 * @return Z-Speed in Body Coordinates [m/s].
 */
static inline float mavlink_msg_emergency_ekf_get_w_B(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field q1 from emergency_ekf message
 *
 * @return Attitude quaternion component 1 [-].
 */
static inline float mavlink_msg_emergency_ekf_get_q1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field q2 from emergency_ekf message
 *
 * @return Attitude quaternion component 2 [-].
 */
static inline float mavlink_msg_emergency_ekf_get_q2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field q3 from emergency_ekf message
 *
 * @return Attitude quaternion component 3 [-].
 */
static inline float mavlink_msg_emergency_ekf_get_q3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field q4 from emergency_ekf message
 *
 * @return Attitude quaternion component 4 [-].
 */
static inline float mavlink_msg_emergency_ekf_get_q4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field roll from emergency_ekf message
 *
 * @return 	Attitude roll expressed as Euler angles, not recommended except for human-readable outputs [rad].
 */
static inline float mavlink_msg_emergency_ekf_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field pitch from emergency_ekf message
 *
 * @return 	Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs [rad].
 */
static inline float mavlink_msg_emergency_ekf_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field yaw from emergency_ekf message
 *
 * @return 	Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs [rad].
 */
static inline float mavlink_msg_emergency_ekf_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field p_0 from emergency_ekf message
 *
 * @return Reference pressure state [mbar/hPa].
 */
static inline float mavlink_msg_emergency_ekf_get_p_0(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field h_0 from emergency_ekf message
 *
 * @return Reference height for pressure readings - constant[m].
 */
static inline float mavlink_msg_emergency_ekf_get_h_0(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field b_s from emergency_ekf message
 *
 * @return Calibration variable - ground bias[m].
 */
static inline float mavlink_msg_emergency_ekf_get_b_s(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Decode a emergency_ekf message into a struct
 *
 * @param msg The message to decode
 * @param emergency_ekf C-struct to decode the message contents into
 */
static inline void mavlink_msg_emergency_ekf_decode(const mavlink_message_t* msg, mavlink_emergency_ekf_t* emergency_ekf)
{
#if MAVLINK_NEED_BYTE_SWAP
	emergency_ekf->time_usec = mavlink_msg_emergency_ekf_get_time_usec(msg);
	emergency_ekf->h_W = mavlink_msg_emergency_ekf_get_h_W(msg);
	emergency_ekf->u_B = mavlink_msg_emergency_ekf_get_u_B(msg);
	emergency_ekf->v_B = mavlink_msg_emergency_ekf_get_v_B(msg);
	emergency_ekf->w_B = mavlink_msg_emergency_ekf_get_w_B(msg);
	emergency_ekf->q1 = mavlink_msg_emergency_ekf_get_q1(msg);
	emergency_ekf->q2 = mavlink_msg_emergency_ekf_get_q2(msg);
	emergency_ekf->q3 = mavlink_msg_emergency_ekf_get_q3(msg);
	emergency_ekf->q4 = mavlink_msg_emergency_ekf_get_q4(msg);
	emergency_ekf->roll = mavlink_msg_emergency_ekf_get_roll(msg);
	emergency_ekf->pitch = mavlink_msg_emergency_ekf_get_pitch(msg);
	emergency_ekf->yaw = mavlink_msg_emergency_ekf_get_yaw(msg);
	emergency_ekf->p_0 = mavlink_msg_emergency_ekf_get_p_0(msg);
	emergency_ekf->h_0 = mavlink_msg_emergency_ekf_get_h_0(msg);
	emergency_ekf->b_s = mavlink_msg_emergency_ekf_get_b_s(msg);
#else
	memcpy(emergency_ekf, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_EMERGENCY_EKF_LEN);
#endif
}
