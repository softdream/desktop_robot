#ifndef __OBSERVATION_H
#define __OBSERVATION_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "utils.h"

namespace aruco
{


template<typename T>
class Observation
{
public:
	using value_type = T;

	Observation()
	{
		dictionary_ = cv::aruco::getPredefinedDictionary( cv::aruco::DICT_4X4_100 );
	}

	Observation( const cv::Mat& K, const cv::Mat& dist, const value_type marker_length ) : K_( K ),	
											     dist_(dist),
											     marker_length_( marker_length )
	{
		dictionary_ = cv::aruco::getPredefinedDictionary( cv::aruco::DICT_4X4_100 );	
	}

	~Observation()
	{

	}

	bool detectMarker( const cv::Mat& image )
	{
		
		std::vector<std::vector<cv::Point2f>> marker_corners;
		std::vector<int> aruco_ids;
		std::vector<cv::Vec<value_type, 3>> rvs, tvs;

		cv::aruco::detectMarkers(image, dictionary_ , marker_corners, aruco_ids);
		cv::aruco::estimatePoseSingleMarkers( marker_corners, marker_length_, K_, dist_, rvs, tvs );		
		if ( aruco_ids.size() != 1 ) return false;	

		// caculate the pose
		cv::Mat r_matrix, t_matrix;
		cv::Rodrigues( rvs[0], r_matrix );
		cv::transpose( tvs[0], t_matrix );
		cv::transpose( t_matrix, t_matrix );
			
		cv::Mat cam_pose_matrix = r_matrix.inv() * ( -t_matrix );
		cv::Mat cam_vec_matrix = r_matrix.inv() * vect_;
		
		pose_[0] = cam_pose_matrix.at<value_type>( 0, 2 );
                pose_[1] = -cam_pose_matrix.at<value_type>( 0, 0 );
                pose_[2] = cam_vec_matrix.at<value_type>(0, 0);

                if ( pose_[2] > 0 ) pose_[2] -= M_PI;
                else pose_[2] += M_PI;

		std::cout<<"pose : ( "<<pose_.transpose()<<" )"<<std::endl;
	
		return true;
	}

	const Eigen::Matrix<value_type, 3, 1>& getPose() const
	{
		return pose_;
	}

private:
	// camera's parameters
	cv::Mat K_, dist_; 
	
	// aruco dictionary
	cv::Ptr<cv::aruco::Dictionary> dictionary_;
	
	// aruco parameters
	value_type marker_length_ = 0;

	cv::Mat vect_ = ( cv::Mat_<value_type>( 3, 1 ) << 0, 0, 1 );

	// pose
	Eigen::Matrix<value_type, 3, 1> pose_ = Eigen::Matrix<value_type, 3, 1>::Zero();
};

}



#endif
