#ifndef ROS_COMM_H_
#define ROS_COMM_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include <ros/ros.h>
#include <common/mavlink.h>
#include "serial_port.h"

#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt64.h"
#include "std_msgs/String.h"

#include "mavctr/acc.h"
#include "mavctr/global_position.h"
#include "mavctr/rc_channels.h"
#include "mavctr/attitude_quad.h"
#include "mavctr/local_position.h"
#include "mavctr/velocity.h"
#include "mavctr/battery.h"
#include "mavctr/State.h"
#include "mavctr/RadioStatus.h"
#include "mavctr/imu.h"
#include "mavctr/Joy.h"
#include "mavctr/position_target_local.h"
#include "mavctr/position_target_global.h"
#include "mavctr/vfr_hud.h"

#include "mavlink_types.h"
#include "sensor_msgs/Joy.h"
//#include "autopilot_interface.h"
#include <sstream>
// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

/**
 * Defines for mavlink_set_position_target_local_ned_t.type_mask
 *
 * Bitmask to indicate which dimensions should be ignored by the vehicle
 *
 * a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of
 * the setpoint dimensions should be ignored.
 *
 * If bit 10 is set the floats afx afy afz should be interpreted as force
 * instead of acceleration.
 *
 * Mapping:
 * bit 1: x,
 * bit 2: y,
 * bit 3: z,
 * bit 4: vx,
 * bit 5: vy,
 * bit 6: vz,
 * bit 7: ax,
 * bit 8: ay,
 * bit 9: az,
 * bit 10: is force setpoint,
 * bit 11: yaw,
 * bit 12: yaw rate
 * remaining bits unused
 *
 * Combine bitmasks with bitwise &
 *
 * Example for position and yaw angle:
 * uint16_t type_mask =
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION &
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
 */

                                                // bit number  876543210987654321
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b0000010111111111


// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------
typedef unsigned long 	UInt64;
typedef long 			Int64;

// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------


// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------


// ----------------------------------------------------------------------------------
//   ROS Interface Class
// ----------------------------------------------------------------------------------
class ROS_Communication{

public:
	ROS_Communication();
	~ROS_Communication();

	ros::NodeHandle comm_nh;

//    ros::Publisher battery_pub, nav_ctrl_status_pub,
//            flight_status_pub, activation_status_pub, test_fre_pub, acc_pub;
//    ros::Publisher gps_pub, att_quad_pub,
//            vel_pub, local_pos_pub,rc_channels_pub;
//    ros::Publisher odem_publisher;

	ros::Publisher flight_status_pub, timestamps_pub, battery_pub,
			radio_status_pub, local_position_ned_pub, global_position_int_pub,
			highres_imu_pub, attitude_pub, gyro_pub,acc_pub,position_target_local_ned_pub,position_target_global_int_pub,
			vfr_hud;

	ros::Subscriber joy_sub,position_target_local_ned_sub, position_target_global_int_sub;

	char writing_status;
    int system_id;
	int autopilot_id;
	int companion_id;
    uint64_t write_count;


	mavlink_rc_channels_override_t rc_override;
	mavlink_manual_control_t		maunal_control;
	mavlink_set_position_target_local_ned_t current_setpoint;

	void publish_heartbeat(mavlink_heartbeat_t heartbeat);
	void publish_sys_status(mavlink_sys_status_t sys_status);
	void publish_battery_status(mavlink_battery_status_t battery_status);
	void publish_radio_status(mavlink_radio_status_t radio_status);
	void publish_local_position_ned(mavlink_local_position_ned_t local_position_nsed);
	void publish_global_position_int(mavlink_global_position_int_t global_position_int);
	void publish_position_target_local_ned(mavlink_position_target_local_ned_t position_target_local_ned);
	void publish_position_target_global_int(mavlink_position_target_global_int_t position_target_global_int);
	void publish_highres_imu(mavlink_highres_imu_t highres_imu);
	void publish_attitude(mavlink_attitude_t attitude);
	void publish_gyro(mavlink_highres_imu_t highres_imu);
	void publish_acc(mavlink_highres_imu_t highres_imu);
	void publish_rc_channels(mavlink_rc_channels_t rc_channels);
	void publish_vfr_hud(mavlink_vfr_hud_t vfr_hud);

	void joyCallback(const mavctr::Joy::ConstPtr& joy);
	void position_target_local_ned_cb(const mavctr::position_target_local::ConstPtr& local_ned);
	void position_target_global_int_cb(const mavctr::position_target_global::ConstPtr& position_target_global_int);

	void write_setpoint();
	int  write_message(mavlink_message_t message);
	void write_manual_control();

	Serial_Port *serial_port;

private:



	//! fcu -> ros
	void mavlink_pub_cb(const mavlink_message_t *mmsg, uint8_t sysid, uint8_t compid);
	//! ros -> fcu
	void mavlink_sub_cb(mavlink_message_t &rmsg);

	//! fcu termination callback
	void terminate_cb();
};
#endif // ROS_COMM_H_
