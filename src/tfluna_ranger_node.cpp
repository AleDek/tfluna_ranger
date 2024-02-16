/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Alessandro De Crescenzo
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <boost/thread.hpp>
#include "i2clink.h"
#include "TFLI2C.h"  // TFLuna-I2C Library v.0.2.0

using namespace std;

static I2cLink i2c_tfluna_left;
static I2cLink i2c_tfluna_right;

static TFLI2C tflI2C_left(&i2c_tfluna_left);
static TFLI2C tflI2C_right(&i2c_tfluna_right);


int main( int argc, char** argv ) {

	ros::init(argc, argv, "tfluna_ranger");;
	ros::NodeHandle n;
	ros::Publisher tfluna_left_pub = n.advertise<sensor_msgs::Range>("/NDT/tfluna_left", 1);
	ros::Publisher tfluna_right_pub = n.advertise<sensor_msgs::Range>("/NDT/tfluna_right", 1);

	sensor_msgs::Range tfluna_left_msg;
	sensor_msgs::Range tfluna_right_msg;

	int16_t  tfAddr = TFL_DEF_ADR;    // default I2C address
	int16_t  tfDist = 0 ;   // distance in centimeters
	int16_t  tfFlux = 0 ;   // signal quality in arbitrary units
	int16_t  tfTemp = 0 ;   // temperature in 0.01 degree Celsius

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



	if(tflI2C_left.initI2c( "/dev/i2c-0"))ROS_INFO("initialized i2c-0 Left");
	else ROS_ERROR("NOT initialized i2c-0 Left");

	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	if(tflI2C_left.scan(tfAddr)) ROS_INFO("Left device found");
    else ROS_ERROR( "Left device NOT Found");


	if(tflI2C_right.initI2c( "/dev/i2c-1"))ROS_INFO("initialized i2c-1 Right");
	else ROS_ERROR("NOT initialized i2c-1 Right");

	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	if(tflI2C_right.scan(tfAddr)) ROS_INFO("Right device found");
    else ROS_ERROR( "Right device NOT Found");




	ros::Rate r(100);

	while(ros::ok()) {

		if( tflI2C_left.getData( tfDist, tfFlux, tfTemp, tfAddr))
        {
			// printf("[L] Dist: %d | Flux: %d | Temp: %d \n", tfDist, tfFlux, tfTemp); 
			tfluna_left_msg.range = float(tfDist/100.00);
        }
        else ROS_ERROR( "Left device reading error");


		if( tflI2C_right.getData( tfDist, tfFlux, tfTemp, tfAddr))
        {
			// printf("[R] Dist: %d | Flux: %d | Temp: %d \n", tfDist, tfFlux, tfTemp); 
			tfluna_right_msg.range = float(tfDist/100.00);
        }
        else ROS_ERROR( "Right device reading error");
		
		
		tfluna_left_pub.publish(tfluna_left_msg);
		tfluna_right_pub.publish(tfluna_right_msg);
		r.sleep();
	}

	return 0;
}
