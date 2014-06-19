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
        	packet1.timestamp = packet_in.timestamp;
        	packet1.height_world = packet_in.height_world;
        	packet1.vel_body_x = packet_in.vel_body_x;
        	packet1.vel_body_y = packet_in.vel_body_y;
        	packet1.vel_body_z = packet_in.vel_body_z;
        	packet1.q_w = packet_in.q_w;
        	packet1.q_x = packet_in.q_x;
        	packet1.q_y = packet_in.q_y;
        	packet1.q_z = packet_in.q_z;
        	packet1.reference_pressure = packet_in.reference_pressure;
        	packet1.phi = packet_in.phi;
        	packet1.theta = packet_in.theta;
        	packet1.psi = packet_in.psi;
        	packet1.reference_height = packet_in.reference_height;
        	packet1.terrain_bias = packet_in.terrain_bias;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_emergency_ekf_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_emergency_ekf_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_emergency_ekf_pack(system_id, component_id, &msg , packet1.timestamp , packet1.height_world , packet1.vel_body_x , packet1.vel_body_y , packet1.vel_body_z , packet1.q_w , packet1.q_x , packet1.q_y , packet1.q_z , packet1.reference_pressure , packet1.phi , packet1.theta , packet1.psi , packet1.reference_height , packet1.terrain_bias );
	mavlink_msg_emergency_ekf_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_emergency_ekf_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.height_world , packet1.vel_body_x , packet1.vel_body_y , packet1.vel_body_z , packet1.q_w , packet1.q_x , packet1.q_y , packet1.q_z , packet1.reference_pressure , packet1.phi , packet1.theta , packet1.psi , packet1.reference_height , packet1.terrain_bias );
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
	mavlink_msg_emergency_ekf_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.height_world , packet1.vel_body_x , packet1.vel_body_y , packet1.vel_body_z , packet1.q_w , packet1.q_x , packet1.q_y , packet1.q_z , packet1.reference_pressure , packet1.phi , packet1.theta , packet1.psi , packet1.reference_height , packet1.terrain_bias );
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
