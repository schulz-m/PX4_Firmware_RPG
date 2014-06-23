/** @file
 *	@brief MAVLink comm protocol testsuite generated from rpg_imu.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef RPG_IMU_TESTSUITE_H
#define RPG_IMU_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_rpg_imu(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

	mavlink_test_rpg_imu(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_rpg_imu(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rpg_imu_t packet_in = {
		93372036854775807ULL,
	}73.0,
	}101.0,
	}129.0,
	}157.0,
	}185.0,
	}213.0,
	};
	mavlink_rpg_imu_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.gyro_x = packet_in.gyro_x;
        	packet1.gyro_y = packet_in.gyro_y;
        	packet1.gyro_z = packet_in.gyro_z;
        	packet1.acc_x = packet_in.acc_x;
        	packet1.acc_y = packet_in.acc_y;
        	packet1.acc_z = packet_in.acc_z;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rpg_imu_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rpg_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rpg_imu_pack(system_id, component_id, &msg , packet1.timestamp , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.acc_x , packet1.acc_y , packet1.acc_z );
	mavlink_msg_rpg_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rpg_imu_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.acc_x , packet1.acc_y , packet1.acc_z );
	mavlink_msg_rpg_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rpg_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rpg_imu_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.acc_x , packet1.acc_y , packet1.acc_z );
	mavlink_msg_rpg_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rpg_imu(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_rpg_imu(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // RPG_IMU_TESTSUITE_H
