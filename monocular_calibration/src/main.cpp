#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "MonocularCalibration.h"

#include <fstream>

using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{
	if ( argc != 3 ) {
		std::cout<<"./CameraCalibration <input_path> <pics_num>"<<std::endl;
		return 0;
	}

	std::string path = argv[1];
	std::string pics_num_str = argv[2];
	int pics_num = std::stoi( pics_num_str );
	std::cout<<"path : "<<path<<std::endl;
	std::cout<<"pics_num : "<<pics_num<<std::endl;

	std::vector<cv::Mat> images;
		
	for( int i = 0; i < pics_num; i ++ ){
		std::string img_path = path + std::to_string( i ) + ".jpg";
		cv::Mat img = cv::imread( img_path );
		images.push_back( img );
	}

	int board_width = 8;
	int board_height = 8;
	
   	MonocularCalibration calib(images, Size(board_width, board_height));
    	calib.onLog();
    	calib.onShowImage();
    	calib.startCalibrate();

    	return 0;
}
