#ifndef __LIDAR_DRIVER_H
#define __LIDAR_DRIVER_H

#include "CYdLidar.h"
#include <functional>

//#include "data_type.h"

namespace lidar
{

using namespace ydlidar;

template<typename T>
using CallBackRef = std::function<void( const T& )>;

class Lidar
{
public:
	Lidar()
	{
		if ( !init() ) {
			std::cerr<<"Cannot Open Lidar Device !"<<std::endl;
		}
	}

	~Lidar()
	{
		release();
	}

	template<typename T>
	void spin( const CallBackRef<T>& cb )
	{
		while ( ret_ && ydlidar::ok() ) {
			bool hard_error = false;

			LaserScan scan_src;

			if ( laser_ptr_->doProcessSimple( scan_src, hard_error ) ) {
				std::cout<<"scan received : stamp = "<<scan_src.stamp<<", points_num = "<<scan_src.points.size()<<", frequency = "<<1.0 / scan_src.config.scan_time<<std::endl;
					
				//for ( int i = 0; i < 100; i ++ ) {
	        	        //        std::cout<<"( "<<scan_src.points.at(i).range<<", "<<scan_src.points.at(i).angle * 180.0 / M_PI<<" ) ";
        		        //}
		                std::cout<<std::endl;

				// callback function
				cb( scan_src );
			}
			else {
				std::cerr<<"Failed to get lidar data !"<<std::endl;
			}
		}
	}

private:
	/*template<typename T>
	void laserScanTypeConvert( const LaserScan& scan_src, T& scan_dst )
	{
		scan_dst.stamp = scan_src.stamp;

		for ( int i = 0; i < scan_dst.size(); i ++ ) {
			scan_dst.points[i].angle = static_cast<typename T::value_type>( scan_src.points.at(i).angle );
			scan_dst.points[i].range = static_cast<typename T::value_type>( scan_src.points.at(i).range );
			scan_dst.points[i].intensity = static_cast<typename T::value_type>( scan_src.points.at(i).intensity );
		}
	}*/


	bool init()
	{
		int argc;
		char** argv;
		
		ydlidar::init(argc, argv);

		if ( !ydlidar::ok() ) return false;

		// 1. lidar configuration
		laser_ptr_ = new CYdLidar();

		laser_ptr_->setSerialPort( port_ );
		laser_ptr_->setSerialBaudrate( baudrate_ );
		laser_ptr_->setFixedResolution( is_fixed_resolution_ );
		laser_ptr_->setReversion( is_reversion_ );
		laser_ptr_->setInverted( false );
		laser_ptr_->setAutoReconnect( true );
		laser_ptr_->setSingleChannel( false );
		laser_ptr_->setLidarType( TYPE_TRIANGLE );
		laser_ptr_->setMaxAngle( 180 );
		laser_ptr_->setMinAngle( -180 );
		laser_ptr_->setMinRange( 30 );
		laser_ptr_->setMaxRange( 1000 );
		laser_ptr_->setScanFrequency( frequency_ );

		ignore_array_.clear();
		laser_ptr_->setIgnoreArray( ignore_array_ );
		laser_ptr_->setIntensity( is_intensiry_ );

		// initialize the lidar
		ret_ = laser_ptr_->initialize();
		if ( !ret_ ) return false;

		// turn on the lidar
		ret_ = laser_ptr_->turnOn();
		if ( !ret_ ) return false;

		if ( !ydlidar::ok() ) return false;

		return true;
	}

	void release()
	{
		laser_ptr_->turnOff();
		laser_ptr_->disconnecting();

		if ( laser_ptr_ != nullptr ) 
			delete laser_ptr_;
	}

private:
	CYdLidar* laser_ptr_ = nullptr;

	const std::string port_ = "/dev/ttyAMA0";
	int baudrate_ = 921600;

	float frequency_ = 8.0;

	std::vector<float> ignore_array_;

	bool is_intensiry_ = true;
	bool is_fixed_resolution_ = false;
	bool is_reversion_ = false;

	bool ret_ = false;

};

}

#endif
