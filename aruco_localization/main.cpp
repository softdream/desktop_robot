#include "observation.h"
#include "data_transport.h"
#include "data_type.h"

#include <iostream>
#include <unistd.h>

#define BUFFER_SIZE 13

char send_buffer[BUFFER_SIZE] = { 0 };

int main()
{
	std::cout<<"------------------------- ARUCO LOCALiZATION ----------------------"<<std::endl;

	// 1. init the aruco detect
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

	// 2. init the camera
	cv::VideoCapture cap(0);

	if ( !cap.isOpened() ) {
                std::cerr<<"Can not open the camera !"<<std::endl;
                return 0;
        }
        std::cout<<"Open the Camera !"<<std::endl;

	cap.set( cv::CAP_PROP_FRAME_WIDTH, 640 );
        cap.set( cv::CAP_PROP_FRAME_HEIGHT, 480 );
        cap.set( cv::CAP_PROP_FPS, 10 );
	

	// 3. init the udp 
	transport::UdpServer udp_srv( LandMarkerPoseProcessPort );	

	while ( 1 ) {
		sensor::MessageType msg;
		auto ret = udp_srv.read( msg );
		std::cout<<"msg = "<<msg<<std::endl;

		// if recved the relocalization command
		if ( ret > 0 && msg == sensor::ReLocalization )	{
			cv::Mat frame;
        		Eigen::Vector3f pose = Eigen::Vector3f::Zero();
			
			int cnt = 0;
			while ( 1 ) {
				usleep( 100000 );

				cap >> frame;
				if ( frame.empty() ) break;

				if ( obs.detectMarker( frame ) ) { // detected the aruco landmarker
	                        	pose = obs.getPose().cast<float>();
        	        	        std::cout<<"------------- get the pose : ( "<<pose.transpose()<<" ) -----------"<<std::endl;

					memset( send_buffer, 0, sizeof( send_buffer ) );
					send_buffer[0] = RelocalizationMsgHeader;
					memcpy( &send_buffer[1], &pose, 12 );
					int ret = udp_srv.write( send_buffer, 13, "127.0.0.1", LocalizationProcessRecverPort );
					std::cout<<" send Relocalized Pose : "<<ret<<std::endl;

					sensor::MessageType msg = sensor::GotRelocalizedPose;
					ret = udp_srv.write( (char*)&msg, sizeof(msg), "127.0.0.1", BehaviorPlannerProcessRecverPort );
					std::cout<<" send GotRelocalizedPose Message : "<<ret<<std::endl;

					break;
				}
				else { // not detect the aruco landmarker
					cnt ++;

					if ( cnt > 200 ) {
						std::cout<<"------------------- TIME OUT ------------------"<<std::endl;
						sensor::MessageType msg = sensor::RelocalizaitonTimeOut;
						int ret = udp_srv.write( (char*)&msg, sizeof(msg), "127.0.0.1", BehaviorPlannerProcessRecverPort );	
						std::cout<<" send RelocalizationTimeOut Message : "<<ret<<std::endl;		

						break;
					}	
				}
			}
		}
	}

	cap.release();


	return 0;
}
