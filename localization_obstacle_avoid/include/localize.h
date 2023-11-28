#ifndef __LOCALIZE_H
#define __LOCALIZE_H

#include "timer.h"
#include "imu_driver.h"
#include "data_transport.h"
#include "ekf_fusion.h"
#include "motor.h"
#include "event_manage.h"
#include "apf_process.h"
#include "path_tracking.h"
#include "ranging_sensor.h"

#ifdef FILE_RECORD
#include <fstream>
#endif

#include <chrono>
#include <thread>

#define RANGING_SENSOR

#define BUFFER_SIZE 13

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
#ifdef FILE_RECORD
		pose_out_.close();
#endif

		if ( udp_serv_ != nullptr ) delete udp_serv_;

#ifdef RANGING_SENSOR
		ranging_sensor_.release();
#endif
	}

	void run()
	{
		if ( !init() ) return;

                std::cout<<"-------------------- START LOCALIZATIOn --------------------"<<std::endl;

                while ( 1 ) {
                        event_instance_.dispatcher();
                }
	}


private:
	// for target path planning and tracking
	void pathPlanningThread( const Vector2& target )
	{
		std::cout<<"--------------- Start Path Planning ---------------"<<std::endl;

		planning::APFProcess<value_type> apf_processor;

		apf_processor.setTargetPose( target );

		int cnt = 0;
		while ( 1 ) {
			usleep( 200000 ); // 5Hz frequency

#ifdef RANGING_SENSOR
			if ( obs_dist_ <= avoid_dist_thresh_ ) {
				usleep( 100000 ); 
                                motor_.cacuRPM( 0.0, 0.0 ); // stop
				
				std::cout<<"--------------- Obatacle ! --------------"<<std::endl;

				sensor::MessageType msg = sensor::ObstacleDetected;
				udp_serv_->send( msg, "127.0.0.1", BehaviorPlannerProcessRecverPort );
				break;
			}
#endif

			auto curr_pose = robot_pose_.head(2); // get the current robot pose
			auto curr_yaw = robot_pose_[2]; // get the current robot yaw angle
			
			auto target_yaw = apf_processor.runApfOnce( curr_pose, obstacles_vec_ ).second;

			auto u = tracking_.cacuControlVector( target_yaw, curr_yaw ); // path tracking
			std::cout<<"curr_yaw = "<<curr_yaw<<", target_yaw = "<<target_yaw<<", u = "<<u.second<<std::endl;

			motor_.cacuRPM( u.first, u.second ); // motor control

			if ( ( curr_pose - target ).norm() < 0.05 ) { // if arrived the goal
				usleep( 100000 );
				motor_.cacuRPM( 0.0, 0.0 );

				std::cout<<"--------------- Arrived The GOAL ! ---------------"<<std::endl;
				sensor::MessageType msg = sensor::ArriveGoalPose;
				udp_serv_->send( msg, "127.0.0.1", BehaviorPlannerProcessRecverPort );	

				break;
			}

			if ( cnt > 50 ) {
				usleep( 100000 );
                                motor_.cacuRPM( 0.0, 0.0 );
				std::cout<<"---------------- TIME OUT ! ---------------"<<std::endl;
				
				sensor::MessageType msg = sensor::Timeout;
				udp_serv_->send( msg, "127.0.0.1", BehaviorPlannerProcessRecverPort );

				break;
			}

			cnt ++;
		}	

		return;
	}

	// for yaw angle path planning and tracking
	void yawTargetPlanningThread( const value_type target_yaw ) 
	{
		std::cout<<"--------------- Start Yaw Target Planning ---------------"<<std::endl;

		int cnt = 0;
		while ( 1 ) {
			usleep( 200000 ); // 5Hz frequency

#ifdef RANGING_SENSOR
                        if ( obs_dist_ <= avoid_dist_thresh_ ) {
                                usleep( 100000 );
                                motor_.cacuRPM( 0.0, 0.0 ); // stop

                                std::cout<<"--------------- Obatacle ! --------------"<<std::endl;

                                sensor::MessageType msg = sensor::ObstacleDetected;
                                udp_serv_->send( msg, "127.0.0.1", BehaviorPlannerProcessRecverPort );
                                break;
                        }
#endif			

			auto curr_yaw = robot_pose_[2]; // get the current robot yaw angle

			auto u = tracking_.cacuControlVectorForRotation( target_yaw, curr_yaw ); // yaw tracking

			motor_.cacuRPM( u.first, u.second ); // motor control

			if ( std::abs( curr_yaw - target_yaw ) <= ( M_PI * 0.04 ) ) { // if arrived the goal yaw angle
				usleep( 100000 );
				motor_.cacuRPM( 0.0, 0.0 );

				std::cout<<"--------------- Arrived The GOAL ! ---------------"<<std::endl;
				sensor::MessageType msg = sensor::ArriveGoalYaw;
                                udp_serv_->send( msg, "127.0.0.1", BehaviorPlannerProcessRecverPort );

                                break;
			}

			if ( cnt > 50 ) {
                                usleep( 100000 );
                                motor_.cacuRPM( 0.0, 0.0 );
                                std::cout<<"------------------- TIME OUT ! ----0--------------"<<std::endl;

				sensor::MessageType msg = sensor::Timeout;
                                udp_serv_->send( msg, "127.0.0.1", BehaviorPlannerProcessRecverPort );

                                break;
                        }

                        cnt ++;
		}

		return;
	}

	bool init()
	{
		// 1. instance of the Receiver
		udp_serv_ = new transport::Receiver( LocalizationProcessRecverPort );

		// 2. imu init & calibration
		if ( !imu_.init() ) return false;
		//imu_.calibration();
		imu_.calibrateGryoZ();

		// 3. motor & encoder init
		motor_.initMotors();
		motor_.initEncoders();

#ifdef RANGING_SENSOR		
		// 4. ranging sensor init
		ranging_sensor_.init();
#endif

		// 5. create a timer
		if ( !timer_.createTimer() ) return false;

		// 6. Epoll : add a timer event
        	event::Event event_timer( timer_.getTimerFd(), EPOLLIN, std::bind( &Localize::timerCallback, this, _1, _2 ), nullptr );
		event_timer.event |= EPOLLET;
	        event_instance_.addEvent( event_timer );
	
        	// 7. Epoll : add a udp event
	        event::Event event_recver( udp_serv_->getSocketFd(), EPOLLIN, std::bind( &Localize::recverCallback, this, _1, _2 ), nullptr );
		event_recver.event |= EPOLLET;
		event_instance_.addEvent( event_recver );

#ifdef RANGING_SENSOR		
		// 8. Epoll : add a laser ranging sensor event
		event::Event event_ranging_event( ranging_sensor_.getFd(), EPOLLIN, std::bind( &Localize::rangingCallback, this, _1, _2 ), nullptr );
		event_ranging_event.event |= EPOLLET;
		event_instance_.addEvent( event_ranging_event );
#endif

		// 9. start the timer
		if ( !timer_.setTimer( 1, 0, 0, 50000000 ) ) return false; // 50ms
	
		return true;
	}

private:

#ifdef RANGING_SENSOR
	void rangingCallback( int fd, void* arg )
	{
		obs_dist_ = ranging_sensor_.getMeasuredVal<value_type>();	
	}
#endif

	void timerCallback( int fd, void* arg )
	{
		//std::cout<<"timer callback ..."<<std::endl;

		// 1. timer handle
		auto ret = timer_.handleRead();

		// 2. motor control
		motor_.motorControl( motor_data_ );

		// 3. get imu data
		value_type gz; // imu measurement
		imu_.getGyroZ( gz );
		gz = -gz;
		std::cout<<"gz = "<<gz<<std::endl;
		
#ifdef FILE_RECORD
		pose_out_ << motor_data_.delta_s<<" "<<motor_data_.delta_angle<<" "<<motor_data_.r_rpm<<" "<<motor_data_.l_rpm<<" "<<gz<<std::endl;
#endif

		// 4. ekf fusion
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
		std::cout<<"pose : ( "<<robot_pose_.transpose()<<" )"<<std::endl;

		// 4.3 update the old value
		pre_gz = gz;
	
		// 4.4 is key pose ?
		if ( poseDiffLargerThan( robot_pose_, last_key_pose_ ) ) {
			last_key_pose_ = robot_pose_;

			udp_serv_->send( robot_pose_, "127.0.0.1", BehaviorPlannerProcessRecverPort );
#ifdef FILE_RECORD
			//pose_out_ << robot_pose_[0]<<" "<<robot_pose_[1]<<" "<<robot_pose_[2]<<std::endl;
#endif
		}

		return;
	}

	void recverCallback( int fd, void* arg )
	{
		std::cout<<"recver callback ..."<<std::endl;
		
		int ret = udp_serv_->read( recv_buffer_, BUFFER_SIZE );

		if ( recv_buffer_[0] == TargetPoseMsgHeader && ret > 0 ) { // target pose
			Vector2 target_pose = Vector2::Zero();
			memcpy( &target_pose, &recv_buffer_[1], 8 );
			std::cout<<"target pose = "<<target_pose.transpose()<<std::endl;
			
			std::thread path_planning_thread( std::bind( &Localize::pathPlanningThread, this, target_pose ) );
			path_planning_thread.detach();
		}
		else if ( recv_buffer_[0] == TargetYawMsgHeader && ret > 0 ) { // target yaw angle
			value_type target_yaw = 0;
			memcpy( &target_yaw, &recv_buffer_[1], 4 );
			std::cout<<"target yaw angle = "<<target_yaw<<std::endl;
			
			std::thread yaw_planning_thread( std::bind( &Localize::yawTargetPlanningThread, this, target_yaw ) );
			yaw_planning_thread.detach();
		}
		else if ( recv_buffer_[0] == RelocalizationMsgHeader && ret > 0 ) { // relocalized pose
			
			Vector3 relocalized_pose = Vector3::Zero();
			memcpy( &relocalized_pose, &recv_buffer_[1], 12 );
			std::cout<<"relocalized pose : ( "<<relocalized_pose.transpose()<<" )"<<std::endl<<std::endl;

			odom_ekf_.resetState( relocalized_pose );
		}
		else if ( recv_buffer_[0] == MoveCommanderHeader && ret > 0 ) { // movement commander
			int command = 0;
			memcpy( &command, &recv_buffer_[1], 4 );

			switch ( command ) {
				case 0x02 : {
					motor_.cacuRPM( 0.0f, 1.0f ); // turn left : low speed
			    		break;		
				}
				case 0x04 : {
					motor_.cacuRPM( 0.0f, 0.0f ); // stop
			    		break;		
				}	
				case 0x05 : {
					motor_.cacuRPM( -0.05f, 0.0f ); // back
					break;	    
				}
	    			default : break;				    
			}
		}
	
		return;
	}

	bool poseDiffLargerThan( const Vector3& pose_now, const Vector3& pose_pre )
	{
		auto pose_diff = pose_now - pose_pre;
		if ( pose_diff.head( 2 ).norm() >= min_dist_diff_ ) return true;

		if ( std::abs( pose_diff[2] ) >= min_angle_diff_ ) return true;

		return false;
	}	

private:
	// devices
	motor::Motor<value_type> motor_;
	imu::MPU6050<value_type> imu_;

#ifdef RANGING_SENSOR
	// added
	sensor::RangingSensor ranging_sensor_;
	value_type obs_dist_ = 0;

	// avoid distance threshold
	constexpr static value_type avoid_dist_thresh_ = 0.05;
#endif

	// timer
	timer::Timer timer_;

	// event management
	event::EpollEvent event_instance_;

	// udp server
	transport::Receiver* udp_serv_;
	char recv_buffer_[BUFFER_SIZE] = { 0 };

	// motor data & imu data
	sensor::MotorData<value_type> motor_data_;
	//sensor::ImuData<value_type> imu_data_;

	// ekf 
	ekf::EKF<value_type> odom_ekf_;

	// ekf variables 
	value_type pre_gz = 0.0;

	// is ekf initialized flag
	bool is_init_ = false;
	
	// robot pose
	Vector3 robot_pose_ = Vector3::Zero();
	Vector3 last_key_pose_ = Vector3::Zero();

	// key poses parameters
	constexpr static value_type min_dist_diff_ = 0.03;
	constexpr static value_type min_angle_diff_ = 0.03490658;

	// path planning
	std::vector<sensor::Obstacle<value_type>> obstacles_vec_;
	planning::Tracking<value_type> tracking_;

	// file storage
#ifdef FILE_RECORD
	std::ofstream pose_out_;
#endif
};	

}


#endif
