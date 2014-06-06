/** @file
 *	@brief MAVLink comm protocol testsuite generated from emergency_ekf.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef EMERGENCY_EKF_TESTSUITE_H
#define EMERGENCY_EKF_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_emergency_ekf(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

	mavlink_test_emergency_ekf(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_emergency_ekf(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_emergency_ekf_t packet_in = {
		93372036854775807ULL,
	}73.0,
	}101.0,
	}129.0,
	}157.0,
	}185.0,
	}213.0,
	}241.0,
	}269.0,
	}297.0,
	}325.0,
	}353.0,
	}381.0,
	}409.0,
	}437.0,
	};
	mavlink_emergency_ekf_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_usec = packet_in.time_usec;
        	packet1.h_W = packet_in.h_W;
        	packet1.u_B = packet_in.u_B;
        	packet1.v_B = packet_in.v_B;
        	packet1.w_B = packet_in.w_B;
        	packet1.q1 = packet_in.q1;
        	packet1.q2 = packet_in.q2;
        	packet1.q3 = packet_in.q3;
        	packet1.q4 = packet_in.q4;
        	packet1.roll = packet_in.roll;
        	packet1.pitch = packet_in.pitch;
        	packet1.yaw = packet_in.yaw;
        	packet1.p_0 = packet_in.p_0;
        	packet1.h_0 = packet_in.h_0;
        	packet1.b_s = packet_in.b_s;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_emergency_ekf_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_emergency_ekf_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_emergency_ekf_pack(system_id, component_id, &msg , packet1.time_usec , packet1.h_W , packet1.u_B , packet1.v_B , packet1.w_B , packet1.q1 , packet1.q2 , packet1.q3 , packet1.q4 , packet1.roll , packet1.pitch , packet1.yaw , packet1.p_0 , packet1.h_0 , packet1.b_s );
	mavlink_msg_emergency_ekf_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_emergency_ekf_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.h_W , packet1.u_B , packet1.v_B , packet1.w_B , packet1.q1 , packet1.q2 , packet1.q3 , packet1.q4 , packet1.roll , packet1.pitch , packet1.yaw , packet1.p_0 , packet1.h_0 , packet1.b_s );
	mavlink_msg_emergency_ekf_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_emergency_ekf_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_emergency_ekf_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.h_W , packet1.u_B , packet1.v_B , packet1.w_B , packet1.q1 , packet1.q2 , packet1.q3 , packet1.q4 , packet1.roll , packet1.pitch , packet1.yaw , packet1.p_0 , packet1.h_0 , packet1.b_s );
	mavlink_msg_emergency_ekf_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_emergency_ekf(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_emergency_ekf(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // EMERGENCY_EKF_TESTSUITE_H
