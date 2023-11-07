#include "event_manage.h"
#include "robot_motion.h"
#include "fsmlist.hpp"

#include "task_planner.h"
#include "goal_generate.h"

#include <thread>

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
planning::TaskPlanner task_planner;
// ----------------------------------------------------------------------------- //

void recvCallback( int fd, void* arg )
{
	std::cout<<"callback ..."<<std::endl;
	
	int ret = udp_srv.read( recv_buffer, BUFFER_SIZE );

	if ( ret == sizeof( sensor::MessageType ) ) { // recved the command message
		sensor::MessageType msg;
		memcpy( &msg, recv_buffer, sizeof( sensor::MessageType ) );
			
		if ( msg == sensor::ArriveGoalPose ) { // arrived the goal pose
			send_event( arrived_goal_pose_event );
			
			if ( relocalization_goal_pose_event.relocalization_goal_pose_flag ) {
				relocalization_goal_pose_event.relocalization_goal_pose_flag = false;

				sleep(1);
				
				sensor::MessageType msg = sensor::ReLocalization;
				int ret = udp_srv.send( msg, "127.0.0.1", LandMarkerPoseProcessPort ); // send relocalization msg
                                std::cout<<" send relocalization command : "<<ret<<std::endl;

				send_event( relocalization_goal_yaw_event );
			}
			else {
				task_planner.staminaValueDecrease();
				task_planner.errorValueIncrease();
			}
		}
		else if ( msg == sensor::ArriveGoalYaw ) { // arrived the goal yaw
        		send_event( arrived_goal_yaw_event );
		
			task_planner.staminaValueDecrease();
			task_planner.errorValueIncrease();
		}
		else if ( msg == sensor::Timeout || msg == sensor::RelocalizaitonTimeOut ) { // time out
			send_event( time_out_event );
		}
		else if ( msg == sensor::GotRelocalizedPose ) { // got the relocalized pose
			send_event( relocalized_pose_detected_event );	

			task_planner.errorValueDecrease();
		}
	}
}

void eventsRecvThread()
{
	// 1. Epoll : add a udp event
        event::Event event_recver( udp_srv.getSocketFd(), EPOLLIN, recvCallback, nullptr );
        event_instance.addEvent( event_recver );


	// 2. Epoll : dispacher
	while ( 1 ) {
        	event_instance.dispatcher();
       	}
}

void taskPlanningThread()
{
	planning::Goal<float> goal_generator;

	while ( 1 ) {
		auto next_status = task_planner.cacuNextStatus();

		if ( next_status == planning::HangOutState ) {
			auto goal_pose = goal_generator.generateTargetPose();
			std::cout<<"goal pose : ( "<<goal_pose.transpose()<<" )"<<std::endl;

			send_event( GoalPose( goal_pose ) );
		}
		else if ( next_status == planning::RestState ) {
		
		}
		else if ( next_status == planning::RelocalizationState ) {
		
		}

		usleep( 500000 );
	}
}

int main()
{
	std::cout<<"--------------------- Task Planner ---------------------"<<std::endl;
	
	// initial fsm state
	fsm_list::start();

	// test
	relocalization_goal_pose_event.relocalization_goal_pose_flag = true;
	send_event( relocalization_goal_pose_event );

	std::thread t1( eventsRecvThread );
	t1.join();

	return 0;
}
