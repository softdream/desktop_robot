#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>


int main()
{
	std::cout<<"------------------------- OBSERVATION TEST ----------------------------"<<std::endl;


	float fx = 604.4518933803821;
	float fy = 605.4001944693676;
	float cx = 317.9623318756531;
	float cy = 228.4397069750446;
	float k1 = 0.1395401810908174;
	float k2 = -0.8106159513271779;
	float p1 = -0.005596355189044073;
	float p2 = -0.002648091051444516;
	float k3 = 0.7691642338731952;

	cv::Mat K = (cv::Mat_<float>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1 ); 
	cv::Mat dist = (cv::Mat_<float>(5, 1) << k1, k2, p1, p2, k3);

	cv::Mat image = cv::imread( "3.jpg" );
	//cv::flip( image, image, 0 );
	
	cv::Ptr<cv::aruco::Dictionary> dictionary =  cv::aruco::getPredefinedDictionary( cv::aruco::DICT_4X4_100 );
	cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

	std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
	std::vector<int> aruco_ids;

	cv::aruco::detectMarkers( image, dictionary, marker_corners, aruco_ids, parameters, rejected_candidates );
	std::cout<<"aruco size = "<<aruco_ids.size()<<std::endl;


	
	cv::waitKey(0);	

	return 0;
}
