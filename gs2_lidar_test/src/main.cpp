#include "CYdLidar.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
using namespace std;
using namespace ydlidar;

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

int main(int argc, char *argv[]) 
{
  	std::cout<<"------------------ GS2 Lidar TEST --------------------"<<std::endl;
	
	std::string port = "/dev/ttyAMA0";


  	ydlidar::init(argc, argv);


  	int baudrate = 921600;


  	if ( !ydlidar::ok() ) {
    		return 0;
  	}


  	bool isSingleChannel = false;
  	bool isTOFLidar = false;

  	float frequency = 8.0;

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
  	
	while ( ret && ydlidar::ok() ) {
      		bool hardError;
      		LaserScan scan;

      		if (laser.doProcessSimple(scan, hardError)) {

			std::cout<<"stamp : "<<scan.stamp<<", points num : "<<scan.points.size()<<std::endl;

			for ( int i = 0; i < 100; i ++ ) {
				std::cout<<"( "<<scan.points.at(i).range<<", "<<scan.points.at(i).angle * 180.0 / M_PI<<" ) ";
			}
			std::cout<<std::endl;

      		}	 
		else {
			std::cerr<<"Failed to get Lidar Data\n";
      		}
  	}

  	laser.turnOff();
  	laser.disconnecting();

  	return 0;
}
