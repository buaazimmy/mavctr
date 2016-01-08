/**
 * @file ros_comm.cpp
 *
 * @brief ROS interface functions
 *
 * Functions for publish and subscribe message from ROS
 *
 *
 */

#include "ros_comm.h"

// ----------------------------------------------------------------------------------
//   ROS Interface Class
// ----------------------------------------------------------------------------------
/**
 * TOPIC List:
 *
 * /mavros/conn_heartbeat
 * /mavros/conn_timeout
 * /mavros/fcu_url
 * /mavros/gcs_url
 * /mavros/gps/frame_id
 * /mavros/imu/angular_velocity_stdev
 * /mavros/imu/frame_id
 * /mavros/imu/linear_acceleration_stdev
 * /mavros/imu/magnetic_stdev
 * /mavros/imu/orientation_stdev
 * /mavros/mission/pull_after_gcs
 * /mavros/plugin_blacklist
 * /mavros/startup_px4_usb_quirk
 * /mavros/sys/min_voltage
 * /mavros/target_component_id
 * /mavros/target_system_id
 * /rosdistro
 * /rosversion
 *
 */
// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}
ROS_Communication::ROS_Communication()
{
	// initialize attributes


    // start ros publisher
    flight_status_pub 				= comm_nh.advertise<std_msgs::UInt8>("/uav_pub/sys_status", 10);
    timestamps_pub 					= comm_nh.advertise<std_msgs::Float32>("/uav_pub/time_stamps", 10);// 10:	queue_size
    battery_pub 					= comm_nh.advertise<mavctr::battery>("/uav_pub/battery_status", 100);// 10:	queue_size
    radio_status_pub 				= comm_nh.advertise<mavctr::RadioStatus>("/uav_pub/radio_status", 10);
    local_position_ned_pub 			= comm_nh.advertise<mavctr::local_position>("/uav_pub/local_position_ned", 10);
    global_position_int_pub			= comm_nh.advertise<mavctr::global_position>("/uav_pub/global_position_int", 10);
    position_target_local_ned_pub	= comm_nh.advertise<mavctr::position_target_local>("/uav_pub/position_target_local_ned", 10);
    position_target_global_int_pub	= comm_nh.advertise<mavctr::position_target_global>("/uav_pub/position_target_global_int", 10);
    highres_imu_pub					= comm_nh.advertise<mavctr::imu>("/uav_pub/highres_imu", 10);
    attitude_pub					= comm_nh.advertise<mavctr::attitude_quad>("/uav_pub/attitude", 10);
    gyro_pub						= comm_nh.advertise<std_msgs::String>("/uav_pub/gyroscope", 10);
    acc_pub 						= comm_nh.advertise<std_msgs::String>("/uav_pub/acceleration", 10);
    vfr_hud							= comm_nh.advertise<mavctr::vfr_hud>("/uav_pub/vfr_hud", 10);

    position_target_local_ned_sub	= comm_nh.subscribe<mavctr::position_target_local>("/uav_sub/position_target_local_ned", 10, &ROS_Communication::position_target_local_ned_cb, this);
    position_target_global_int_sub	= comm_nh.subscribe<mavctr::position_target_global>("/uav_sub/position_target_global_int", 10, &ROS_Communication::position_target_global_int_cb, this);
    joy_sub 						= comm_nh.subscribe<mavctr::Joy>("joy", 10, &ROS_Communication::joyCallback, this);
//
//    //extend pub
//
//    att_quad_pub = comm_nh.advertise<dji_sdk::attitude_quad>("attitude_quad", 10);
//    gps_pub = comm_nh.advertise<dji_sdk::global_position>("global_position", 10);
//    local_pos_pub = comm_nh.advertise<dji_sdk::local_position>("local_position", 10);
//    vel_pub = comm_nh.advertise<dji_sdk::velocity>("velocity", 10);
//    odem_publisher = comm_nh.advertise<nav_msgs::Odometry>("odom",10);
//
//    rc_channels_pub = comm_nh.advertise<dji_sdk::rc_channels>("rc_channels",10);
	rc_override.chan1_raw = 0;
	rc_override.chan2_raw = 0;
	rc_override.chan3_raw = 0;
	rc_override.chan4_raw = 0;
	rc_override.chan5_raw = 0;
	rc_override.chan6_raw = 0;
	rc_override.chan7_raw = 0;
	rc_override.chan8_raw = 0;
	rc_override.target_component = 1;
	rc_override.target_system = 1;

	//init manual control

	maunal_control.x = 0;
	maunal_control.y = 0;
	maunal_control.z = 0;
	maunal_control.r = 0;
	maunal_control.buttons = 0;
	maunal_control.target = 1;
}

ROS_Communication::~ROS_Communication(){;}

void ROS_Communication::publish_heartbeat(mavlink_heartbeat_t heartbeat)
{
	 std_msgs::UInt8 msg;
	 msg.data=heartbeat.base_mode;

	 flight_status_pub.publish(msg);
}
void ROS_Communication::publish_sys_status(mavlink_sys_status_t sys_status)
{
//	mavctr::State msg;
}
void ROS_Communication::publish_battery_status(mavlink_battery_status_t battery_status)
{
//	 std_msgs::String msg;
//	 std::stringstream ss;
//	 ss  << "battery_status: " << (int)battery_status.battery_remaining << std::endl;
//	 msg.data = ss.str();
//
//	 battery_pub.publish(msg);

	mavctr::battery msg;
	msg.battery_function = battery_status.battery_function;
	msg.battery_remaining = battery_status.battery_remaining;
	msg.current_battery = battery_status.current_battery;
	msg.current_consumed = battery_status.current_consumed;
	msg.energy_consumed = battery_status.energy_consumed;
	msg.id = battery_status.id;
	msg.temperature = battery_status.temperature;
	msg.type = battery_status.type;

	battery_pub.publish(msg);
}
void ROS_Communication::publish_radio_status(mavlink_radio_status_t radio_status)
{
	mavctr::RadioStatus msg;
	msg.header.stamp.nsec = 0;
	msg.fixed = radio_status.fixed;
	msg.noise = radio_status.noise;
	msg.remnoise = radio_status.remnoise;
	msg.remrssi = radio_status.remrssi;
	msg.rssi = radio_status.rssi;
	msg.rxerrors = radio_status.rxerrors;
	msg.txbuf = radio_status.txbuf;

	radio_status_pub.publish(msg);
}
void ROS_Communication::publish_local_position_ned(mavlink_local_position_ned_t local_position_ned)
{
	mavctr::local_position msg;
	msg.x 		= local_position_ned.x;
	msg.y		= local_position_ned.y;
	msg.height	= local_position_ned.z;
	msg.velx	= local_position_ned.vx;
	msg.vely	= local_position_ned.vy;
	msg.velz	= local_position_ned.vz;

	local_position_ned_pub.publish(msg);

}
void ROS_Communication::publish_global_position_int(mavlink_global_position_int_t global_position_int)
{
	mavctr::global_position msg;
	msg.ts = global_position_int.time_boot_ms;
	msg.lat = global_position_int.lat;
	msg.lon = global_position_int.lon;
	msg.alti = global_position_int.hdg;
	msg.height = global_position_int.relative_alt;

	global_position_int_pub.publish(msg);
}
void ROS_Communication::publish_position_target_local_ned(mavlink_position_target_local_ned_t position_target_local_ned)
{
	mavctr::position_target_local msg;
	msg.ts = position_target_local_ned.time_boot_ms;
	msg.x = position_target_local_ned.x;
	msg.y = position_target_local_ned.y;
	msg.z = position_target_local_ned.z;
	msg.vx = position_target_local_ned.vx;
	msg.vy = position_target_local_ned.vy;
	msg.vz = position_target_local_ned.vz;
	msg.afx = position_target_local_ned.afx;
	msg.afy = position_target_local_ned.afy;
	msg.afz = position_target_local_ned.afz;
	msg.yaw = position_target_local_ned.yaw;
	msg.yaw_rate = position_target_local_ned.yaw_rate;
	msg.type_mask = position_target_local_ned.type_mask;
	msg.coordinate_frame = position_target_local_ned.coordinate_frame;

	position_target_local_ned_pub.publish(msg);
}
void ROS_Communication::publish_position_target_global_int(mavlink_position_target_global_int_t position_target_global_int)
{
	mavctr::position_target_global msg;
	msg.ts = position_target_global_int.time_boot_ms;
	msg.lat_int = position_target_global_int.lat_int;
	msg.lon_int = position_target_global_int.lon_int;
	msg.vx = position_target_global_int.vx;
	msg.vy = position_target_global_int.vy;
	msg.vz = position_target_global_int.vz;
	msg.afx = position_target_global_int.afx;
	msg.afy = position_target_global_int.afy;
	msg.afz = position_target_global_int.afz;
	msg.yaw = position_target_global_int.yaw;
	msg.yaw_rate = position_target_global_int.yaw_rate;
	msg.type_mask = position_target_global_int.type_mask;
	msg.coordinate_frame = position_target_global_int.coordinate_frame;

	position_target_global_int_pub.publish(msg);
}
void ROS_Communication::publish_highres_imu(mavlink_highres_imu_t highres_imu)
{
	mavctr::imu msg;
	msg.wx = highres_imu.xgyro;
	msg.wy = highres_imu.ygyro;
	msg.wz = highres_imu.zgyro;
	msg.ax = highres_imu.xacc;
	msg.ay = highres_imu.yacc;
	msg.az = highres_imu.zacc;
	msg.mx = highres_imu.xmag;
	msg.my = highres_imu.ymag;
	msg.mz = highres_imu.zmag;
	msg.abs_pressure = highres_imu.abs_pressure;
	msg.diff_pressure = highres_imu.diff_pressure;
	msg.pressure_alt = highres_imu.pressure_alt;
	msg.temperature = highres_imu.temperature;
	msg.ts = highres_imu.time_usec;
	msg.fields_updated = highres_imu.fields_updated;

	highres_imu_pub.publish(msg);
}
void ROS_Communication::publish_attitude(mavlink_attitude_t attitude)
{
	mavctr::attitude_quad msg;
	msg.wx = attitude.rollspeed;
	msg.wy = attitude.pitchspeed;
	msg.wz = attitude.yawspeed;

	attitude_pub.publish(msg);
}
void ROS_Communication::publish_gyro(mavlink_highres_imu_t highres_imu)
{
	;
}
void ROS_Communication::publish_acc(mavlink_highres_imu_t highres_imu)
{
	;
}
void ROS_Communication::publish_rc_channels(mavlink_rc_channels_t rc_channels)
{
	mavctr::rc_channels msg;
	msg.roll = rc_channels.chan1_raw;
	msg.pitch = rc_channels.chan2_raw;
	msg.throttle = rc_channels.chan3_raw;
	msg.yaw = rc_channels.chan4_raw;
	msg.mode = rc_channels.chan5_raw;
	msg.gear_up = rc_channels.chan6_raw;

//	radio_status_pub.publish(msg);
}

void ROS_Communication::publish_vfr_hud(mavlink_vfr_hud_t vfr_hud)
{
	;
}

void ROS_Communication::position_target_local_ned_cb(const mavctr::position_target_local::ConstPtr& local_ned)
{
	write_setpoint();
}
void ROS_Communication::position_target_global_int_cb(const mavctr::position_target_global::ConstPtr& position_target_global_int)
{

}
void ROS_Communication::joyCallback(const mavctr::Joy::ConstPtr& joy)
{
	maunal_control.x=1000*joy->axes[0];
	maunal_control.y=1000*joy->axes[1];
	maunal_control.z=500+500*joy->axes[3];
	maunal_control.r=1000*joy->axes[2];
//	maunal_control.buttons = 32;              //16:offboard mode
//	write_manual_control();
//joy->axes[0];
//	joy->axes[linear_];

}
void ROS_Communication::write_setpoint()
{
	//   PACK PAYLOAD
	mavlink_set_position_target_local_ned_t sp = current_setpoint;

	// double check some system parameters
	if ( not sp.time_boot_ms )
		sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
	sp.target_system    = system_id;
	sp.target_component = autopilot_id;

	//   ENCODE
	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);

	//   WRITE
	int len = write_message(message);
	// check the write
	if ( not len > 0 )
		fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	return;
}
// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int ROS_Communication::write_message(mavlink_message_t message)
{
	// do the write
	int len = serial_port->write_message(message);

	// book keep
	write_count++;

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Write MANUAL_CONTROL Message
// ------------------------------------------------------------------------------
void ROS_Communication::write_manual_control(){

	//   ENCODE
	mavlink_message_t message;
	mavlink_msg_manual_control_encode(system_id,companion_id,&message, &maunal_control);

	//   WRITE
	int len = write_message(message);
	// check the write
	if ( not len > 0 )
		fprintf(stderr,"WARNING: could not send write_manual_control \n");

	return;
}

