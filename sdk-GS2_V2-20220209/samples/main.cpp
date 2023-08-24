﻿/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, EAIBOT, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
#include "CYdLidar.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
using namespace std;
using namespace ydlidar;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_driver.lib")
#endif

#define DEFAULT_TIMEOUT 2000

result_t initRecvPort(serial::Serial **_serial,std::string port_name,int baudrate){
    if (!(*_serial)) {
        *_serial = new serial::Serial(port_name, baudrate,
                                     serial::Timeout::simpleTimeout(DEFAULT_TIMEOUT));
    }

    if((*_serial)->isOpen()){
        return RESULT_OK;
    }
    if (!(*_serial)->open()) {
        return RESULT_FAIL;
    }

    return  RESULT_OK;
}

int main(int argc, char *argv[]) {
  printf("__   ______  _     ___ ____    _    ____  \n");
  printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
  printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
  printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
  printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
  printf("\n");
  fflush(stdout);
  std::string port = "/dev/ttyAMA0";


  ydlidar::init(argc, argv);


  //int baudrate = 230400;
  int baudrate = 921600;

  printf("Baudrate:\n");

  if (!ydlidar::ok()) {
    return 0;
  }


  bool isSingleChannel = false;
  bool isTOFLidar = false;
//  std::string input_channel;
//  std::string input_tof;
//  printf("Whether the Lidar is one-way communication[yes/no]:");
//  std::cin >> input_channel;
//  std::transform(input_channel.begin(), input_channel.end(),
//                 input_channel.begin(),
//  [](unsigned char c) {
//    return std::tolower(c);  // correct
//  });
//
//  if (input_channel.find("yes") != std::string::npos) {
//    isSingleChannel = true;
//  }


//  printf("Whether the Lidar is a TOF Lidar [yes/no]:");
//  std::cin >> input_tof;
//  std::transform(input_tof.begin(), input_tof.end(),
//                 input_tof.begin(),
//  [](unsigned char c) {
//    return std::tolower(c);  // correct
//  });

//  if (input_tof.find("yes") != std::string::npos) {
//    isTOFLidar = true;
//  }

//  if (!ydlidar::ok()) {
//    return 0;
//  }

  std::string input_frequency;

  float frequency = 8.0;
  if (!ydlidar::ok()) {
    return 0;
  }

  CYdLidar laser;
  //<! lidar port
  laser.setSerialPort(port);
  //<! lidar baudrate
  laser.setSerialBaudrate(baudrate);

  //<! fixed angle resolution
  laser.setFixedResolution(false);
  //<! rotate 180
  laser.setReversion(false); //rotate 180
  //<! Counterclockwise
  laser.setInverted(false);//ccw
  laser.setAutoReconnect(true);//hot plug
  //<! one-way communication
  laser.setSingleChannel(false);

  //<! tof lidar
  laser.setLidarType(isTOFLidar ? TYPE_TOF : TYPE_TRIANGLE);
  //unit: °
  laser.setMaxAngle(180);
  laser.setMinAngle(-180);

  //unit: m
  laser.setMinRange(30);
  laser.setMaxRange(1000);

  //unit: Hz
  laser.setScanFrequency(frequency);
  std::vector<float> ignore_array;
  ignore_array.clear();
  laser.setIgnoreArray(ignore_array);
  bool isIntensity = true;
  laser.setIntensity(isIntensity);

  bool ret = laser.initialize();

  if (ret) {
    ret = laser.turnOn();
  }

  result_t isOk = RESULT_OK;
  while (ret && ydlidar::ok())
  {
      bool hardError;
      LaserScan scan;

      if (laser.doProcessSimple(scan, hardError))
      {
          /*fprintf(stdout, "Scan received[%lu]: %u ranges is [%f]Hz\n",
                  scan.stamp,
                  (unsigned int)scan.points.size(),
                  1.0 / scan.config.scan_time);
          fflush(stdout);
		*/
	     //printf( "stamp : %lu, point num : %d\n", scan.stamp, scan.points.size() );

		std::cout<<"stamp : "<<scan.stamp<<", points num : "<<scan.points.size()<<std::endl;

		for ( int i = 0; i < 100; i ++ ) {
			std::cout<<"( "<<scan.points.at(i).range<<", "<<scan.points.at(i).angle * 180.0 / M_PI<<" ) ";
		}
		std::cout<<std::endl;

//          for (size_t i=0; i<scan.points.size(); ++i)
 //         {
  //            printf("%d %f %f\n", i, scan.points.at(i).range, scan.points.at(i).angle * 180.0 / M_PI);
    //      }
      //    printf("\n");
      } else {
          fprintf(stderr, "Failed to get Lidar Data\n");
          fflush(stderr);
      }
  }

  laser.turnOff();
  laser.disconnecting();

  return 0;
}
