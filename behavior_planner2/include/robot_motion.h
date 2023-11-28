#ifndef __ROBOT_MOTION_H
#define __ROBOT_MOTION_H

#include <iostream>
#include "tinyfsm.hpp"

#include "data_transport.h"
#include "data_type.h"
#include <Eigen/Dense>


// --------------------------------- event declarations --------------------------------- //
struct ArrivedGoalPose : tinyfsm::Event {  };

struct ArrivedGoalYaw : tinyfsm::Event {  };

struct TimeOut : tinyfsm::Event {  };

struct GoalPose : tinyfsm::Event 
{
	GoalPose()
	{
	
	}

	GoalPose( const Eigen::Vector2f& goal_pose_ ) : goal_pose( goal_pose_ )
	{
	
	}
		
	Eigen::Vector2f goal_pose = Eigen::Vector2f::Zero();
}; 

struct GoalYaw : tinyfsm::Event 
{
	GoalYaw()
	{
	
	}

	GoalYaw( const float goal_yaw_ ) : goal_yaw( goal_yaw_ )
	{
	
	} 

	float goal_yaw = 0.0;
};

struct RelocalizationGoalPose : tinyfsm::Event 
{  
	bool relocalization_goal_pose_flag = false;
};


struct RelocalizationGoalYaw : tinyfsm::Event 
{  
	bool relocalization_goal_yaw_flag = false;	
};


struct RelocalizedPoseDetected : tinyfsm::Event {  };

struct ObstacleDetected : tinyfsm::Event {  };

// --------------------- Robot Motion ( FSM base class ) declarations ----------------- //
class RobotMotion : public tinyfsm::Fsm<RobotMotion>
{
public:
	// default reaction for unhandled evnets
        void react( const tinyfsm::Event& )
        {

        };

	virtual void react( const ArrivedGoalPose& e );

        virtual void react( const ArrivedGoalYaw& e );

        virtual void react( const TimeOut& e );
 
	virtual void react ( const GoalPose& e );

	virtual void react ( const GoalYaw& e );

	virtual void react( const RelocalizationGoalPose& e );

	virtual void react( const RelocalizationGoalYaw& e );

	virtual void react( const RelocalizedPoseDetected& e );

	virtual void react( const ObstacleDetected& e );

	// entry actions in some states
        virtual void entry( void );

	// exit actions in some states
        virtual void exit( void );

//protected:
//	transport::UdpClient udp_client_ = transport::UdpClient( LocalizationProcessRecverPort, "127.0.0.1" );
};


#endif
