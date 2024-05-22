
// C library headers
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <iostream>
#include <ros/ros.h>
#include "sensor_msgs/Range.h"


#define _DEBUG_ON 0

void deserialize(const uint8_t* buffer, float* dists){
    //if(buffer[0] =='$')
    uint32_t temp =0;
    temp |= buffer[1]<<24;
    temp |= buffer[2]<<16;
    temp |= buffer[3]<<8;
    temp |= buffer[4];
    dists[0] = *((float*)&temp);
    temp=0;
    temp |= buffer[5]<<24;
    temp |= buffer[6]<<16;
    temp |= buffer[7]<<8;
    temp |= buffer[8];
    dists[1] = *((float*)&temp);
}

void configure_serial(int & serial_port, const char* port_name, speed_t baud){
    serial_port = open(port_name, O_RDWR);

	// Check for errors
    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    struct termios tty;
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }
    /* refer to: https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/*/
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)
    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;      //n of byte that block read()
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tfluna_serial_node");
    ros::NodeHandle _nh;

    std::string _device_port;
    bool _verbose = false;

    if( !_nh.getParam("/tfluna/serial_port", _device_port)) { 
        _device_port = "/dev/ttyACM0";
        ROS_ERROR("error loading parameter /tfluna/serial_port");
    }

    if( !_nh.getParam("/tfluna/print_on", _verbose)) { 
        _verbose = false;
        ROS_ERROR("error loading parameter /tfluna/print_on");
    }

    int _serial_port;
    uint8_t _rcv_buff[9];
    int _num_bytes;
    float _dists[2];

    ros::Publisher tfluna_left_pub = _nh.advertise<sensor_msgs::Range>("/NDT/tfluna_left", 1);
	ros::Publisher tfluna_right_pub = _nh.advertise<sensor_msgs::Range>("/NDT/tfluna_right", 1);

	sensor_msgs::Range tfluna_left_msg;
	sensor_msgs::Range tfluna_right_msg;

	tfluna_left_msg.radiation_type = sensor_msgs::Range::INFRARED;
	tfluna_left_msg.field_of_view = 0.04;
	tfluna_left_msg.min_range = 0.12;
	tfluna_left_msg.max_range = 8;
	tfluna_left_msg.header.frame_id = "tfluna_left";


	tfluna_right_msg.radiation_type = sensor_msgs::Range::INFRARED;
	tfluna_right_msg.field_of_view = 0.04;
	tfluna_right_msg.min_range = 0.12;
	tfluna_right_msg.max_range = 8;
	tfluna_right_msg.header.frame_id = "tfluna_right";

    configure_serial(_serial_port,_device_port.c_str(),B115200);
   
    ros::Rate rate(1000);
    
    while(ros::ok()){
        _num_bytes = read(_serial_port, &_rcv_buff[0], 1);
        if(_rcv_buff[0] == '$'){
            for(int i = 1;i<25;i++){
                _num_bytes = read(_serial_port, &_rcv_buff[i], 1);
            }
            deserialize(_rcv_buff,_dists);
            /*print for debug*/
            //printf("RAW message: %s \n", _rcv_buff);
            if(_verbose) printf("Dists: [%f %f]\n",_dists[0],_dists[1]);

            /*publish on Ros recived wrench*/
            if(_dists[0]>=0) tfluna_left_msg.range  = _dists[0];
            else ROS_ERROR( "Left device reading error");

            if(_dists[1]>=0) tfluna_right_msg.range = _dists[1];
            else ROS_ERROR( "Right device reading error");

            //header.stamp = ros::Time::now();
            tfluna_left_pub.publish(tfluna_left_msg);
		    tfluna_right_pub.publish(tfluna_right_msg);
        }
        /*check for recive errors*/
        if (_num_bytes < 0) {
            ROS_ERROR("Error reading serial: %s", strerror(errno));
            return 1;
        }
        //ros::spinOnce();
        rate.sleep();
    }

    close(_serial_port);
    return 0;
}