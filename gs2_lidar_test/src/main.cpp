#include <iostream>
#include "lidar_driver.h"

#include "utils.h"

// ----------------------------- GLOBAL DATA ------------------------------- //
sensor::ScanContainerF container;

void lidarCallback( const LaserScan& scan )
{
	std::cout<<"lidar data call back ..."<<std::endl;

	Utils::laserScan2Container( scan, container );
	std::cout<<"scan container size = "<<container.size()<<std::endl;

	Utils::displayOneFrameScan( container );
}


int main(int argc, char *argv[]) 
{
  	std::cout<<"------------------ GS2 Lidar TEST --------------------"<<std::endl;

	lidar::Lidar gs2_lidar;
	gs2_lidar.spin<LaserScan>( lidarCallback );	

  	return 0;
}
