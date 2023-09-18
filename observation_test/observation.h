#ifndef __OBSERVATION_H
#define __OBSERVATION_H

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

namespace obs
{

template<typename T>
class Observation
{
public:
	using value_type = T;

	Observation()
        {
                dictionary_ = cv::aruco::getPredefinedDictionary( cv::aruco::DICT_4X4_50 );
        }

	Observation( const cv::Mat& K, const cv::Mat& dist, const value_type marker_len ) : K_( K ), dist_( dist ), marker_len_( marker_len )
	{
		dictionary_ = cv::aruco::getPredefinedDictionary( cv::aruco::DICT_4X4_50 );
	}
	
	~Observation()
	{
	
	}

	void detectMarker( const cv::Mat& image )
	{
		std::vector<std::vector<cv::Point2f>> marker_corners;
		std::vector<int> aruco_ids;
		std::vector<cv::Vec3d> rvs, tvs;

		cv::aruco::detectMarkers( image, dictionary_, marker_corners, aruco_ids );
		std::cout<<"aruco size = "<<aruco_ids.size()<<std::endl;
		cv::aruco::estimatePoseSingleMarkers( marker_corners, marker_len_, K_, dist_, rvs, tvs );
		std::cout<<"size = "<<aruco_ids.size()<<std::endl;

		// draw 
		cv::aruco::drawDetectedMarkers( image, marker_corners, aruco_ids );
		std::cout<<"......."<<std::endl;

		// for every marekr
		for ( uint8_t i = 0; i < aruco_ids.size(); i ++ ) {
			std::cout<<"rotation vector: " << std::endl<<rvs[i]<<std::endl;
			std::cout<<"transport vector : "<<std::endl<<tvs[i]<<std::endl;

			// caculate the pose
			cv::Mat cam_pose_mat, cam_vec_mat;
			cv::Mat vect = ( cv::Mat_<float>( 3, 1 ) << 0, 0, 1 );

			cv::Mat r_mat, t_mat;
			cv::Rodrigues( rvs[i], r_mat );
			cv::transpose( tvs[i], t_mat );
			cv::transpose( t_mat, t_mat );
		
			cam_pose_mat = r_mat.inv() * ( -t_mat );
			cam_vec_mat = r_mat.inv() * vect;

			std::cout<<"Camera Position : "<<cam_pose_mat.t()<<std::endl;
			std::cout<<"Camera Direction : "<<cam_vec_mat.t()<<std::endl;
		}

		cv::imshow( "aruco", image );
	}

private:
	// camera's parameters
	cv::Mat K_;
	cv::Mat dist_;

	// aruco dictionary
	cv::Ptr<cv::aruco::Dictionary> dictionary_;

	cv::Mat marker_img_;

	// aruco parameters
	value_type marker_len_ = 0;
};

}

#endif
