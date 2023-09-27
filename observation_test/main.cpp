#include "observation.h"
#include <iostream>

#include <chrono>

int main()
{
	std::cout<<"---------------- OBSERVATION TEST ------------------"<<std::endl;

	using value_type = double;

	value_type fx = 604.4518933803821;
        value_type fy = 605.4001944693676;
        value_type cx = 317.9623318756531;
        value_type cy = 228.4397069750446;
        value_type k1 = 0.1395401810908174;
        value_type k2 = -0.8106159513271779;
        value_type p1 = -0.005596355189044073;
        value_type p2 = -0.002648091051444516;
        value_type k3 = 0.7691642338731952;
	
	value_type marker_len = 0.095;

	cv::Mat K = (cv::Mat_<value_type>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
        cv::Mat dist = (cv::Mat_<value_type>(5, 1) << k1, k2, p1, p2, k3);	

	aruco::Observation<value_type> obs( K, dist, marker_len );
	
	for ( int i = 0; i <= 4; i ++ ) {

		cv::Mat image = cv::imread( "./data3/" + std::to_string( i ) + ".jpg" );
	
		auto befor = std::chrono::steady_clock::now();
		obs.detectMarker( image );
		auto after = std::chrono::steady_clock::now();


		double duration = std::chrono::duration<double, std::milli>( after - befor ).count();
		std::cout<<"duration = "<<duration<<std::endl;
	
		cv::waitKey(0);
	}

	cv::waitKey(0);

	return 0;
}
