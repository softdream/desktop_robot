#ifndef __LOCALIZE_H
#define __LOCALIZE_H

#include "timer.h"
#include "imu_driver.h"
#include "data_transport.h"
#include "ekf_fusion.h"
#include "motor.h"
#include "event_manage.h"
#include "observation.h"

#include <thread>


namespace chassis
{

using namespace std::placeholders;

template<typename T>
class Localize
{
public:
	using value_type = T;

        using Vector2 = typename Eigen::Matrix<value_type, 2, 1>;
        using Vector3 = typename Eigen::Matrix<value_type, 3, 1>;
        using Vector4 = typename Eigen::Matrix<value_type, 4, 1>;
        using Vector6 = typename Eigen::Matrix<value_type, 6, 1>;

        using Matrix3x3 = typename Eigen::Matrix<value_type, 3, 3>;
		
	Localize()
	{
	
	}

	~Localize()
	{
		if ( udp_serv_ != nullptr ) delete udp_serv_;
	}

	void run()
	{
		// 1. localization thread
		std::thread localize_thread( std::bind( &Localize::localizationThread, this ) );
		
		// 2. camera aruco localization thread
		
		
		localize_thread.join();
	}

private:
	void visionUpdateThread()
	{
		
	}

private:
	void localizationThread()
	{
		if ( !init() ) return;

		while ( 1 ) {
			event_instance_.dispatcher();
		}
	}

	bool init()
	{
		// 1. instance of the Receiver
		udp_serv_ = new transport::Receiver<50>( 2333 );

		// 2. imu init & calibration
		if ( !imu_.init() ) return false;
		imu_.calibration();
	
		// 3. motor & encoder init
		motor_.initMotors();
		motor_.initEncoders();

		// 4. create a timer
		if ( !timer_.createTimer() ) return false;

		// 5. Epoll : add a timer event
        	event::Event event_timer( timer_.getTimerFd(), EPOLLIN, std::bind( &Localize::timerCallback, this, _1, _2 ), nullptr );
	        event_instance_.addEvent( event_timer );
	
        	// 6. Epoll : add a udp event
	        event::Event event_recver( udp_serv_->getSocketFd(), EPOLLIN, std::bind( &Localize::recverCallback, this, _1, _2 ), nullptr );
		event_instance_.addEvent( event_recver );
	
		// 7. start the timer
		if ( !timer_.setTimer( 1, 0, 0, 50000000 ) ) return false; // 50ms
	
		return true;
	}

private:
	void timerCallback( int fd, void* arg )
	{
		std::cout<<"timer callback ..."<<std::endl;

		// 1. timer handle
		auto ret = timer_.handleRead();

		// 2. motor control
		motor_.motorControl( motor_data_ );

		// 3. get imu data
		imu_.getImuData( imu_data_ );

		// 4. ekf fusion
		value_type gz = -imu_data_.gyro_z;

		if ( motor_data_.r_rpm == 0 || motor_data_.l_rpm == 0 ) gz = 0.0;

		if ( !is_init_ ) {
			pre_gz = gz;	
			is_init_ = true;
	
			return;			
		}

		// 4.1 state predict
		odom_ekf_.predict( Vector2( motor_data_.delta_s, motor_data_.delta_angle ) );

		// 4.2 state update by imu
		odom_ekf_.update( 0.025 * ( pre_gz + gz ) ); // delta_t * ( pre_gz + gz ) / 2

		// update the robot pose
		robot_pose_ = odom_ekf_.getStateX();

		// 4.3 update the old value
		pre_gz = gz;
	}

	void recverCallback( int fd, void* arg )
	{
		std::cout<<"recver callback ..."<<std::endl;
		
		int command = 0;
		udp_serv_->receive( command );
	
		switch ( command ) {
			case 0x01 : { // forwad
				motor_.cacuRPM( 0.01f, 0.0f );
				break;				
			}
			case 0x02 : { // turn left
				motor_.cacuRPM( 0.0f, 0.1f );
				break;
			}
			case 0x02 : { // turn left
                                motor_.cacuRPM( 0.0f, -0.1f );
                                break;
                        }	    
			default : break;
		}
	}

private:
	// devices
	motor::Motor<value_type> motor_;
	imu::MPU6050<value_type> imu_;

	// timer
	timer::Timer timer_;

	// event management
	event::EpollEvent event_instance_;

	// udp server
	transport::Receiver<50>* udp_serv_;

	// motor data & imu data
	sensor::MotorData<value_type> motor_data_;
	sensor::ImuData<value_type> imu_data_;

	// ekf 
	ekf::EKF<value_type> odom_ekf_;

	// ekf variables 
	value_type pre_gz = 0.0;

	// is ekf initialized flag
	bool is_init_ = false;
	
	// robot pose
	Vector3 robot_pose_ = Vector3::Zero();
};	

}


#endif
