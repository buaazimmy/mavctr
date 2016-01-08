/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file autopilot_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */

#define __need_timeval
// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "autopilot_interface.h"
	int count_write=0;
timeval t_start,t_end;
long last_time=0;
long count=0,total_time=0;
// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
extern uint64_t get_time_usec();


// ----------------------------------------------------------------------------------
//   Setpoint Helper Functions
// ----------------------------------------------------------------------------------

// choose one of the next three

/*
 * Set target local ned position
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target XYZ locations
 * in the Local NED frame, in meters.
 */
void
set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.x   = x;
	sp.y   = y;
	sp.z   = z;

	printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);

}

/*
 * Set target local ned velocity
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target VX VY VZ
 * velocities in the Local NED frame, in meters per second.
 */
void
set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.vx  = vx;
	sp.vy  = vy;
	sp.vz  = vz;

	//printf("VELOCITY SETPOINT UVW = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);

}

/*
 * Set target local ned acceleration
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target AX AY AZ
 * accelerations in the Local NED frame, in meters per second squared.
 */
void
set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp)
{

	// NOT IMPLEMENTED
	fprintf(stderr,"set_acceleration doesn't work yet \n");
	throw 1;


	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION &
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.afx  = ax;
	sp.afy  = ay;
	sp.afz  = az;
}

// the next two need to be called after one of the above

/*
 * Set target local ned yaw
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw
 * in the Local NED frame, in radians.
 */
void
set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;

	sp.yaw  = yaw;

	printf("POSITION SETPOINT YAW = %.4f \n", sp.yaw);

}

/*
 * Set target local ned yaw rate
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw rate
 * in the Local NED frame, in radians per second.
 */
void
set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE ;

	sp.yaw_rate  = yaw_rate;
}
int measure_interval()
{
	gettimeofday(&t_end,NULL);
	last_time = (t_end.tv_sec - t_start.tv_sec)*1000000 +  t_end.tv_usec - t_start.tv_usec;

	t_start = t_end;
	printf("interval time:%f \n",(float)last_time/1000000);
	if(last_time>10000000)
		return 0;
	total_time += last_time;
	count++;
//	printf("Count:%ld \t Average Freq:%ld \n",count,total_time/1000000);
}

// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_Interface::
Autopilot_Interface(Serial_Port *serial_port_)
{
	// initialize attributes
	ros_comm.write_count = 0;

	reading_status = 0;      // whether the read thread is running
	ros_comm.writing_status = 0;      // whether the write thread is running
	control_status = 0;      // whether the autopilot is in offboard control mode
	time_to_exit   = false;  // flag to signal thread exit

	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id

	ros_comm.system_id    = 0; // system id
	ros_comm.autopilot_id = 0; // autopilot component id
	ros_comm.companion_id = 0; // companion computer component id

	current_messages.sysid  = ros_comm.system_id;
	current_messages.compid = ros_comm.autopilot_id;

	ros_comm.serial_port = serial_port_; // serial port management object

}

Autopilot_Interface::
~Autopilot_Interface()
{}


// ------------------------------------------------------------------------------
//   Update Setpoint
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
	ros_comm.current_setpoint = setpoint;
}


/**Safety Arm
 *
 */
void Autopilot_Interface::safety_arm(void)
{
	mavlink_set_mode_t	mav_mode;
	mav_mode.target_system = ros_comm.system_id;
	mav_mode.base_mode =	MAV_MODE_FLAG_SAFETY_ARMED 			//128, /* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. | */
//							|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED	//1, /* 0b00000001 Reserved for future use. | */
//							| MAV_MODE_FLAG_TEST_ENABLED		//2, /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
//							| MAV_MODE_FLAG_AUTO_ENABLED		//4, /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
							| MAV_MODE_FLAG_GUIDED_ENABLED		//8, /* 0b00001000 guided mode enabled, system flies e / mission items. | */
//							| MAV_MODE_FLAG_STABILIZE_ENABLED	//16, /* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */
//							| MAV_MODE_FLAG_HIL_ENABLED			//32, /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
							| MAV_MODE_FLAG_MANUAL_INPUT_ENABLED//64, /* 0b01000000 remote control input is enabled. | */
							;
	mavlink_message_t msg_set_mode;
	mavlink_msg_set_mode_encode(ros_comm.system_id,ros_comm.companion_id,&msg_set_mode,&mav_mode);
	int len = ros_comm.write_message(msg_set_mode);
}
// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_messages()
{
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;
	bool a = not received_all and not time_to_exit;

//	mavlink_message_t message1;
//	success = ros_comm.serial_port->read_message(message1);
//	printf("time_to_exit = %d\n",a);
//	printf("success = %d\n",success);
	// Blocking wait for new data
	while ( not received_all and not time_to_exit )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		printf("time_to_exit = %d\n",a);
		success = ros_comm.serial_port->read_message(message);
		printf("success = %d\n",success);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{

			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;

			// Handle Message ID
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT://1Hz
				{
					//printf("MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					ros_comm.publish_heartbeat(current_messages.heartbeat);
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS://0.5Hz
				{
					//printf("MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
					ros_comm.publish_sys_status(current_messages.sys_status);

					break;
				}

				case MAVLINK_MSG_ID_BATTERY_STATUS://0.5Hz
				{
					//printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					current_messages.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
					ros_comm.publish_battery_status(current_messages.battery_status);
					break;
				}

				case MAVLINK_MSG_ID_RADIO_STATUS://
				{
					//printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
					mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
					current_messages.time_stamps.radio_status = get_time_usec();
					this_timestamps.radio_status = current_messages.time_stamps.radio_status;
					ros_comm.publish_radio_status(current_messages.radio_status);
					break;
				}

				case MAVLINK_MSG_ID_LOCAL_POSITION_NED://1.5Hz
				{
					//printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
					mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
					current_messages.time_stamps.local_position_ned = get_time_usec();
					this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
					ros_comm.publish_local_position_ned(current_messages.local_position_ned);
					break;
				}

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT://1.5Hz
				{
					//printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
					current_messages.time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
					ros_comm.publish_global_position_int(current_messages.global_position_int);
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED://1.5Hz
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
					mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
					current_messages.time_stamps.position_target_local_ned = get_time_usec();
					this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
					ros_comm.publish_position_target_local_ned(current_messages.position_target_local_ned);
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT://1.5Hz
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
					mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
					current_messages.time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
					ros_comm.publish_position_target_global_int(current_messages.position_target_global_int);
					break;
				}

				case MAVLINK_MSG_ID_HIGHRES_IMU://1Hz
				{
					//printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
					mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
					current_messages.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
					ros_comm.publish_highres_imu(current_messages.highres_imu);
//					measure_interval();
					break;
				}

				case MAVLINK_MSG_ID_ATTITUDE://10Hz
				{
					//printf("MAVLINK_MSG_ID_ATTITUDE\n");
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					ros_comm.publish_attitude(current_messages.attitude);
					break;
				}
				case MAVLINK_MSG_ID_RC_CHANNELS:
				{
//					printf("MAVLINK_MSG_ID_RC_CHANNELS\n");
//					printf("=================================");
					mavlink_msg_rc_channels_decode(&message, &(current_messages.rc_channels));
					current_messages.time_stamps.rc_channels = get_time_usec();
					this_timestamps.rc_channels = current_messages.time_stamps.rc_channels;
					ros_comm.publish_rc_channels(current_messages.rc_channels);
					printf("rc_chan3=%d\n",current_messages.rc_channels.chan3_raw);
					break;
				}
				case MAVLINK_MSG_ID_VFR_HUD:
				{
					mavlink_msg_vfr_hud_decode(&message, &(current_messages.vfr_hud));
					current_messages.time_stamps.vfr_hud = get_time_usec();
					this_timestamps.vfr_hud= current_messages.time_stamps.vfr_hud;
	//				ros_comm.vfr_hud(current_messages.vfr_hud);
					break;
				}

				default:
				{
					printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}


			} // end: switch msgid

		} // end: if read message

		// Check for receipt of all items
		received_all =
				this_timestamps.heartbeat                  &&
				this_timestamps.sys_status                 &&
//				this_timestamps.battery_status             &&
//				this_timestamps.radio_status               &&
				this_timestamps.local_position_ned         &&
//				this_timestamps.global_position_int        &&
//				this_timestamps.position_target_local_ned  &&
				this_timestamps.position_target_global_int &&
				this_timestamps.highres_imu                &&
				this_timestamps.attitude                   ;

		// give the write thread time to use the port
		if ( ros_comm.writing_status > false )
			usleep(100); // look for components of batches at 10kHz

	} // end: while not received all

	return;
}

//// ------------------------------------------------------------------------------
////   Write Message
//// ------------------------------------------------------------------------------
//int
//Autopilot_Interface::
//write_message(mavlink_message_t message)
//{
//	// do the write
//	int len = serial_port->write_message(message);
//
//	// book keep
//	write_count++;
//
//	// Done!
//	return len;
//}
// ------------------------------------------------------------------------------
//   Write RC override Message
// ------------------------------------------------------------------------------
void Autopilot_Interface::write_RCoverride(){

	//   ENCODE
	mavlink_message_t message;
	mavlink_msg_rc_channels_override_encode(ros_comm.system_id,ros_comm.companion_id,&message, &ros_comm.rc_override);

	//   WRITE
	int len = ros_comm.write_message(message);
	// check the write
	if ( not len > 0 )
		fprintf(stderr,"WARNING: could not send rc_channels_override \n");

	return;
}

//// ------------------------------------------------------------------------------
////   Write MANUAL_CONTROL Message
//// ------------------------------------------------------------------------------
//void Autopilot_Interface::write_manual_control(){
//
//	//   ENCODE
//	mavlink_message_t message;
//	mavlink_msg_manual_control_encode(ros_comm.system_id,ros_comm.companion_id,&message, &ros_comm.maunal_control);
//
//	//   WRITE
//	int len = ros_comm.write_message(message);
//	// check the write
//	if ( not len > 0 )
//		fprintf(stderr,"WARNING: could not send write_manual_control \n");
//
//	return;
//}
// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
//void
//Autopilot_Interface::
//write_setpoint()
//{
//	//   PACK PAYLOAD
//	mavlink_set_position_target_local_ned_t sp = ros_comm.current_setpoint;
//
//	// double check some system parameters
//	if ( not sp.time_boot_ms )
//		sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
//	sp.target_system    = ros_comm.system_id;
//	sp.target_component = ros_comm.autopilot_id;
//
//	//   ENCODE
//	mavlink_message_t message;
//	mavlink_msg_set_position_target_local_ned_encode(ros_comm.system_id, ros_comm.companion_id, &message, &sp);
//
//	//   WRITE
//	int len = ros_comm.write_message(message);
//	// check the write
//	if ( not len > 0 )
//		fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
//	//	else
//	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", ros_comm.write_count, position_target.x, position_target.y, position_target.z);
//
//	return;
//}

// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
enable_offboard_control()
{

	// Should only send this command once
	if ( control_status == false )
	{
		printf("ENABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to go off-board
		int success = toggle_offboard_control( true );

		// Check the command was written
		if ( success )
			control_status = true;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");


	} // end: if not offboard_status
}


// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
disable_offboard_control()
{

	// Should only send this command once
	if ( control_status == true )
	{
		printf("DISABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to stop off-board
		int success = toggle_offboard_control( false );

		// Check the command was written
		if ( success )
			control_status = false;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if offboard_status

}


// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
toggle_offboard_control( bool flag )
{

	// Prepare command for off-board mode
	mavlink_command_long_t com;
	com.target_system    = ros_comm.system_id;
	com.target_component = ros_comm.autopilot_id;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation     = true;
	com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(ros_comm.system_id, ros_comm.companion_id, &message, &com);

	// Send the message
	int len = ros_comm.serial_port->write_message(message);

	// Done!
	return len;
}


// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start()
{
	int result;

	// --------------------------------------------------------------------------
	//   CHECK SERIAL PORT
	// --------------------------------------------------------------------------

	if ( not ros_comm.serial_port->status == 1 ) // SERIAL_PORT_OPEN
	{
		fprintf(stderr,"ERROR: serial port not open\n");
		throw 1;
	}
	// --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------

	printf("START READ THREAD \n");

	result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
	if ( result ) throw result;

	// now we're reading messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   CHECK FOR MESSAGES
	// --------------------------------------------------------------------------

	printf("CHECK FOR MESSAGES\n");

	while ( not current_messages.sysid )
	{
		if ( time_to_exit )
			return;
		usleep(500000); // check at 2Hz
	}

	printf("Found\n");

	// now we know autopilot is sending messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   GET SYSTEM and COMPONENT IDs
	// --------------------------------------------------------------------------

	// This comes from the heartbeat, which in theory should only come from
	// the autopilot we're directly connected to it.  If there is more than one
	// vehicle then we can't expect to discover id's like this.
	// In which case set the id's manually.

	// System ID
	if ( not ros_comm.system_id )
	{
		ros_comm.system_id = current_messages.sysid;
		printf("GOT VEHICLE SYSTEM ID: %i\n", ros_comm.system_id );
	}

	// Component ID
	if ( not ros_comm.autopilot_id )
	{
		ros_comm.autopilot_id = current_messages.compid;
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", ros_comm.autopilot_id);
		printf("\n");
	}


	// --------------------------------------------------------------------------
	//   GET INITIAL POSITION
	// --------------------------------------------------------------------------

	// Wait for initial position ned
	while ( not ( current_messages.time_stamps.local_position_ned &&
				  current_messages.time_stamps.attitude            )  )
	{
		if ( time_to_exit )
			return;
		usleep(500000);
	}

	// copy initial position ned
	Mavlink_Messages local_data = current_messages;
	initial_position.x        = local_data.local_position_ned.x;
	initial_position.y        = local_data.local_position_ned.y;
	initial_position.z        = local_data.local_position_ned.z;
	initial_position.vx       = local_data.local_position_ned.vx;
	initial_position.vy       = local_data.local_position_ned.vy;
	initial_position.vz       = local_data.local_position_ned.vz;
	initial_position.yaw      = local_data.attitude.yaw;
	initial_position.yaw_rate = local_data.attitude.yawspeed;

	printf("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x, initial_position.y, initial_position.z);
	printf("INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
	printf("\n");

	// we need this before starting the write thread
	safety_arm();

	//init rc_override
	ros_comm.rc_override.target_component = ros_comm.autopilot_id;
	ros_comm.rc_override.target_system = ros_comm.system_id;
//	rc_override.chan3_raw = 2000;

	//init manual control
	ros_comm.maunal_control.target = ros_comm.system_id;


	// --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
	printf("START WRITE THREAD \n");

	result = pthread_create( &write_tid, NULL, &start_autopilot_interface_write_thread, this );
	if ( result ) throw result;
	// wait for it to be started

	while ( not ros_comm.writing_status )
		usleep(100000); // 10Hz

	// now we're streaming setpoint commands
	printf("\n");


	// Done!
	return;

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
stop()
{

	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;


	// wait for exit
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);
	// now the read and write threads are closed

	// disarm
	mavlink_set_mode_t	mav_mode;
	mav_mode.target_system = ros_comm.system_id;
	mav_mode.base_mode = 81;
	mavlink_message_t msg_set_mode;
	mavlink_msg_set_mode_encode(ros_comm.system_id,ros_comm.companion_id,&msg_set_mode,&mav_mode);
	int len = ros_comm.write_message(msg_set_mode);

	printf("\n");

	// still need to close the serial_port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_read_thread()
{

	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_write_thread(void)
{
	if ( not ros_comm.writing_status == false )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		write_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
handle_quit( int sig )
{

	disable_offboard_control();

	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}

}



// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_thread()
{
	reading_status = true;

	while ( not time_to_exit )
	{
		read_messages();
		usleep(100000); // Read batches at 10Hz
	}

	reading_status = false;

	return;
}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_thread(void)
{
	// signal startup
	ros_comm.writing_status = 2;

	// prepare an initial setpoint, just stay put
	mavlink_set_position_target_local_ned_t sp;
	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
				   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.vx       = 0.0;
	sp.vy       = 0.0;
	sp.vz       = 0.0;
	sp.yaw_rate = 0.0;

	// set position target
	ros_comm.current_setpoint = sp;

	// write a message and signal writing
//	write_setpoint();
	ros_comm.writing_status = true;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while ( not time_to_exit )
	{
		usleep(250000);   // Stream at 40Hz
		count_write++;
//		usleep(4000);   // Stream at 25Hz
//		write_setpoint();

//		write_RCoverride();
//		rc_override.chan3_raw = 2000;
//		if(rc_override.chan3_raw>2500)
//			rc_override.chan3_raw = 2500;
//		ROS_INFO("RC_ch3:%d",rc_override.chan3_raw );
//if((count_write>0)&(count_write<400))


		ros_comm.maunal_control.buttons = 32;//16:offboard mode
		ros_comm.write_manual_control();
		ROS_INFO("Manual control.x:%d",ros_comm.maunal_control.x);
		ROS_INFO("Manual control.y:%d",ros_comm.maunal_control.y);
		ROS_INFO("Manual control.z:%d",ros_comm.maunal_control.z);
		ROS_INFO("Manual control.r:%d",ros_comm.maunal_control.r);
	}
	// signal end
	ros_comm.writing_status = false;

	return;

}

// End Autopilot_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_autopilot_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_read_thread();

	// done!
	return NULL;
}

void*
start_autopilot_interface_write_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_write_thread();

	// done!
	return NULL;
}



