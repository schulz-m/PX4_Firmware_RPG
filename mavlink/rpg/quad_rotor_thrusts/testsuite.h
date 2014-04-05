/** @file
 *	@brief MAVLink comm protocol testsuite generated from quad_rotor_thrusts.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef QUAD_ROTOR_THRUSTS_TESTSUITE_H
#define QUAD_ROTOR_THRUSTS_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_quad_rotor_thrusts(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

	mavlink_test_quad_rotor_thrusts(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_quad_rotor_thrusts(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_quad_rotor_thrusts_t packet_in = {
		93372036854775807ULL,
	}73.0,
	}101.0,
	}129.0,
	}157.0,
	};
	mavlink_quad_rotor_thrusts_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.thrust_1 = packet_in.thrust_1;
        	packet1.thrust_2 = packet_in.thrust_2;
        	packet1.thrust_3 = packet_in.thrust_3;
        	packet1.thrust_4 = packet_in.thrust_4;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_quad_rotor_thrusts_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_quad_rotor_thrusts_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_quad_rotor_thrusts_pack(system_id, component_id, &msg , packet1.timestamp , packet1.thrust_1 , packet1.thrust_2 , packet1.thrust_3 , packet1.thrust_4 );
	mavlink_msg_quad_rotor_thrusts_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_quad_rotor_thrusts_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.thrust_1 , packet1.thrust_2 , packet1.thrust_3 , packet1.thrust_4 );
	mavlink_msg_quad_rotor_thrusts_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_quad_rotor_thrusts_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_quad_rotor_thrusts_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.thrust_1 , packet1.thrust_2 , packet1.thrust_3 , packet1.thrust_4 );
	mavlink_msg_quad_rotor_thrusts_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_quad_rotor_thrusts(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_quad_rotor_thrusts(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // QUAD_ROTOR_THRUSTS_TESTSUITE_H
