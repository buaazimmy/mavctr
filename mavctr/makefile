all: mavlink_control

obj = mavlink_control.cpp serial_port.cpp autopilot_interface.cpp ros_comm.cpp

mavlink_control: clean mavlink_control.cpp
	g++ -I../include $(obj) -o mavlink_control -lpthread

clean:
	 rm -rf *o mavlink_control
