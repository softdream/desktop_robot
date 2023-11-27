#include "event_manage.h"
#include "robot_motion.h"
#include "fsmlist.hpp"

#include "task_planner.h"
#include "goal_generate.h"
#include "timer.h"

#include <thread>
#include <mutex>
#include <condition_variable>

#define BUFFER_SIZE 12

// --------------------------------- GLOBAL DATA ------------------------------- //
event::EpollEvent event_instance;
transport::Receiver udp_srv( BehaviorPlannerProcessRecverPort );

char recv_buffer[BUFFER_SIZE] = { 0 };

// global fsm events
ArrivedGoalPose arrived_goal_pose_event;
ArrivedGoalYaw arrived_goal_yaw_event;
TimeOut time_out_event;
RelocalizationGoalPose relocalization_goal_pose_event;
RelocalizationGoalYaw relocalization_goal_yaw_event;
RelocalizedPoseDetected relocalized_pose_detected_event;

// for task planning
planning::TaskPlanner<float> task_planner( "/groupdata/share/ddf/Test/desktop_robot-master/behavior_planner3/model/behavior_planner.mlp" );
std::mutex event_mux;
std::condition_variable event_cv;
bool is_event_occured = false;

timer::Timer timer_instance;

bool is_entertainment_needed = false;
std::mutex entertainment_mux;
std::condition_variable entertainment_cv;
bool is_entertainment_ready = false;
// ----------------------------------------------------------------------------- //

void recvCallback( int fd, void* arg )
{
	std::cout<<"callback ..."<<std::endl;
	
	int ret = udp_srv.read( recv_buffer, BUFFER_SIZE );

	if ( ret == sizeof( sensor::MessageType ) ) { // recved the command message
		sensor::MessageType msg;
		memcpy( &msg, recv_buffer, sizeof( sensor::MessageType ) );
	
		switch ( msg ) {		
			case sensor::ArriveGoalPose : { // arrived the goal pose
				send_event( arrived_goal_pose_event );
		
				// if need relocalization	
				if ( relocalization_goal_pose_event.relocalization_goal_pose_flag ) {
					relocalization_goal_pose_event.relocalization_goal_pose_flag = false;
	
					sleep(1);
					
					sensor::MessageType msg = sensor::ReLocalization;
					int ret = udp_srv.send( msg, "127.0.0.1", LandMarkerPoseProcessPort ); // send relocalization msg
	                                std::cout<<" send relocalization command : "<<ret<<std::endl;
	
					send_event( relocalization_goal_yaw_event );
				}
				// if need entertainment
				else if ( is_entertainment_needed ) {
					sleep( 1 );

					send_event( GoalYaw( M_PI ) );
				}
				// don't need relocalization
				else {
					is_event_occured = true;	
					event_cv.notify_one();
				}

				break;
			}
			case sensor::ArriveGoalYaw : { // arrived the goal yaw
        			send_event( arrived_goal_yaw_event );
				
				// if need entertainment
                                if ( is_entertainment_needed ) {
					sleep( 1 );

					is_entertainment_ready = true;
					entertainment_cv.notify_one();
				}
				else {
					is_event_occured = true;
					event_cv.notify_one();
				}	

				break;
			}
			case sensor::Timeout : { // time out
				send_event( time_out_event );

				is_event_occured = true;
				event_cv.notify_one();
				
				break;
			}
			case sensor::RelocalizaitonTimeOut : {
				send_event( time_out_event );
	
				is_event_occured = true;
				event_cv.notify_one();

				break;				     
			}
			case sensor::GotRelocalizedPose : { // got the relocalized pose
				send_event( relocalized_pose_detected_event );	
			
				// ------------ task planner ----------- //
				task_planner.errorValueReset();

				is_event_occured = true;
				event_cv.notify_one();

				break;
			}
			case sensor::ObstacleDetected : { // detected a obstacle
				send_event( ObstacleDetected() );

				// ------------ task planner ----------- //
				task_planner.setDetectedObstacle();

				is_event_occured = true;
				event_cv.notify_one();

				break;
			}
			case sensor::IsKeyPose : { // got one key pose
				// ------------ task planner ----------- //
				task_planner.staminavalueDecrease( task_planner.randomIncrement() );	
				task_planner.moodValueIncrease( task_planner.randomIncrement() * 0.2 );
				//task_planner.entertainmentValueIncrease( task_planner.randomIncrement() * 0.3 );
				task_planner.errorValueIncrease();

				break;			 
			} 
			default : break;
		} //switch
	}
}

void timerCallback( int fd, void* arg )
{
	std::cout<<"timer callback ..."<<std::endl;

	// 1. timer handle
        auto ret = timer_instance.handleRead();

	task_planner.moodValueDecrease( task_planner.randomIncrement() * 0.8 );

	return;
}

void eventsRecvThread()
{
	if ( !timer_instance.createTimer() ) return ;

	event::Event event_timer( timer_instance.getTimerFd(), EPOLLIN, timerCallback, nullptr );
        event_timer.event |= EPOLLET;
        event_instance.addEvent( event_timer );

	// 1. Epoll : add a udp event
        event::Event event_recver( udp_srv.getSocketFd(), EPOLLIN, recvCallback, nullptr );
        event_recver.event |= EPOLLET;
	event_instance.addEvent( event_recver );

	if ( !timer_instance.setTimer( 1, 0, 1, 0 ) ) return ;

	// 2. Epoll : dispacher
	while ( 1 ) {
        	event_instance.dispatcher();
       	}
}

// ----------------------------- STATUS PROCESSES THREADS ------------------------- //
void restStatusProcessThread()
{
	// 1. emoj display
	
	// 2. attribution management
	int time_duration = task_planner.randomTimeDuration();
	std::cout<<"Time Duration : "<<time_duration<<std::endl;

	for ( int i = time_duration; i > 0; i -- ) {
		std::cout<<i<<std::endl;

		task_planner.moodValueIncrease( task_planner.randomIncrement() );
		task_planner.staminaValueIncrease( task_planner.randomIncrement() * 0.7 );
		task_planner.entertainmentValueIncrease( task_planner.randomIncrement() * 0.8 );

		usleep( 500000 );
	}

	is_event_occured = true;
        event_cv.notify_one();
	return;
}

void entertainmentStatusProcessThread()
{
	// 1. emoj display 
	
	// 2. move base
	is_entertainment_needed = true;
	send_event( GoalPose( Eigen::Vector2f( 10, 0 ) ) );

	while ( 1 ) {
		std::unique_lock lk( entertainment_mux );
		entertainment_cv.wait( lk, []{ return is_entertainment_ready; } );
	
		is_entertainment_ready = false;
		break;
	}
	is_entertainment_needed = false;

	// 3. attribution management
	int time_duration = task_planner.randomTimeDuration();
	std::cout<<"Time Duration : "<<time_duration<<std::endl;

        for ( int i = time_duration; i > 0; i -- ) {
                std::cout<<i<<std::endl;
	
		task_planner.moodValueIncrease( task_planner.randomIncrement() * 0.2 );
                task_planner.staminavalueDecrease( task_planner.randomIncrement() * 0.4 );
                task_planner.entertainmentValueDecrease( task_planner.randomIncrement() * 0.8 );	
	
		usleep( 500000 );
	}

	is_event_occured = true;
        event_cv.notify_one();
        return;
}

void hangoutStatusProcessThread()
{
	// 1. emoj display
	

	// 2. move base
	planning::Goal<float> goal_generator;
	auto goal_pose = goal_generator.generateTargetPose();
	std::cout<<"goal pose : ( "<<goal_pose.transpose()<<" )"<<std::endl;

	send_event( GoalPose( goal_pose ) );
}

void enjoymentStatusProcessThread()
{
	// 1. emoj display


        // 2. attribution management
	int time_duration = task_planner.randomTimeDuration();
        std::cout<<"Time Duration : "<<time_duration<<std::endl;

        for ( int i = time_duration; i > 0; i -- ) {
                std::cout<<i<<std::endl;

                task_planner.moodValueIncrease( task_planner.randomIncrement() * 0.1 );
                task_planner.entertainmentValueDecrease( task_planner.randomIncrement() * 0.3 );

                usleep( 500000 );
        }

        is_event_occured = true;
        event_cv.notify_one();
        return;
}

void relocalizationStatusProcessThread()
{
	// 1. emoj display


        // 2. attribution management
	relocalization_goal_pose_event.relocalization_goal_pose_flag = true;
        send_event( relocalization_goal_pose_event );
}

// -------------------------------------------------------------------------------- //

void taskPlanningThread()
{

	while ( 1 ) {
		{
			std::cout<<"---------- Start Task Planning ------------"<<std::endl;
			auto next_status = task_planner.cacuNextStatus();
		
			std::cout<<"next status = "<<next_status<<std::endl;
		
			switch ( next_status ) {
				case planning::HangOutState : {
					std::cout<<"---------------- Hangout Status ----------------"<<std::endl;
					
					hangoutStatusProcessThread();
					
					break;
				}
				case planning::RestState : {
					std::cout<<"------------------ Rest Status -----------------"<<std::endl;

					std::thread rest_thread( restStatusProcessThread );
					rest_thread.join();

					break;		 
				}
				case planning::EntertainmentState : {
					std::cout<<"-------------- Entertainment State --------------"<<std::endl;

					std::thread entertainment_thread( entertainmentStatusProcessThread );
					entertainment_thread.join();

    					break;					
				}
				case planning::EnjoymentState : {
					std::cout<<"---------------- Enjoyment State ----------------"<<std::endl;

					std::thread enjoyment_thread( enjoymentStatusProcessThread );
					enjoyment_thread.join();

					break;				
				}
				case planning::RelocalizationState : {
					std::cout<<"-------------- ReLocalization State -------------"<<std::endl;
					
					relocalizationStatusProcessThread();

					break;				     
				}
				default : break;
			}

			std::unique_lock lk( event_mux );
                        event_cv.wait( lk, []{ return is_event_occured; } );
	
			is_event_occured = false;
			std::cout<<"here ..."<<std::endl;
		}

		usleep( 500000 );
	}
}

int main()
{
	std::cout<<"--------------------- Task Planner ---------------------"<<std::endl;
	
	// initial fsm state
	fsm_list::start();


	std::thread t1( eventsRecvThread );
	std::thread t2( taskPlanningThread );

	t1.join();
	t2.join();

	return 0;
}
