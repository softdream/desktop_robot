#ifndef __UTILS_H
#define __UTILS_H


#include "CYdLidar.h"
#include "scan_container.h"
#include <opencv2/opencv.hpp>


#define WIDTH 600
#define HEIGHT 600

#define CENTER_X 300
#define CENTER_Y 300

class Utils
{
public:
	template<typename T>
	static void angleNormalize( T&& angle )
	{
		if ( angle >= M_PI ) angle -= 2 * M_PI;

		if ( angle <= -M_PI ) angle += 2 * M_PI;
	}


	template<typename T>
	static void laserScan2Container( const LaserScan& scan, sensor::ScanContainer<T>& container )
	{
		container.clear();

		for ( int i = 0; i < scan.points.size(); i ++ ) {
			auto dist = static_cast<T>( scan.points[i].range ) * 0.001;

			// judgement
			if ( dist > 0.03 && dist < 0.3 ) {
				auto theta = scan.points[i].angle;

				angleNormalize( theta );

				auto pt_x = dist * ::cos( theta );
				auto pt_y = dist * ::sin( theta );

				container.add( typename sensor::ScanContainer<T>::type( pt_x, pt_y ) );
			}
		}
	}

	template<typename T>
	static void displayOneFrameScan( const sensor::ScanContainer<T>& container )
	{
		cv::Mat image = cv::Mat::zeros( WIDTH, HEIGHT, CV_8UC3 );

		cv::line( image, cv::Point( CENTER_X, 0 ), cv::Point( CENTER_X ,HEIGHT ), cv::Scalar( 0, 255, 0 ), 2 );
		cv::line( image, cv::Point( 0, CENTER_Y ), cv::Point( WIDTH, CENTER_Y ), cv::Scalar( 0, 255, 0 ), 2 );

		for ( int i = 0; i < container.size(); i ++ ) {
			auto pt = container.at( i );
		
			cv::Point pt_img( pt[0] * 1000 + CENTER_X, pt[1] * 1000 + CENTER_Y );
			cv::circle( image, pt_img, 3, cv::Scalar(0, 0, 255), -1 );
		}

		cv::imshow( "scan", image );
		cv::waitKey(5);
	}
};

#endif
