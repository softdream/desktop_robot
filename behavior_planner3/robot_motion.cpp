#include "robot_motion.h"

// ------------------------ GLOBAL DATA ----------------------- //
transport::UdpClient udp_client_ = transport::UdpClient( LocalizationProcessRecverPort, "127.0.0.1" );
// ---------------- State Forward Declarations ----------------- //
class Stop;
class Rotation;
class Navigation;
class Back;

// ------------------------ State : Stop ----------------------- //
class Stop : public RobotMotion
{
public:
	void entry( void ) override
        {
                std::cout<<"Stop State !"<<std::endl;
        }

	void react( const RelocalizationGoalPose& e ) override
	{
		auto action = [=]() {
			std::cout<<" send Relocalization Goal Pose "<<std::endl;
			
			Eigen::Vector2f required_relocalization_goal_pose( 0.25, 0.0 );
			int ret = udp_client_.write( required_relocalization_goal_pose, TargetPoseMsgHeader );
			std::cout<<" send required relocalization goal pose : "<<ret<<std::endl;
		};

		transit<Navigation>( action );
	}

	void react( const RelocalizationGoalYaw& e ) override
	{
		auto action = [=]() {
                        std::cout<<" send Relocalization Goal Yaw "<<std::endl;

			int command = 0x02;
			int ret = udp_client_.write( command, MoveCommanderHeader );
			std::cout<<" send relocalization Rotation Command : "<<ret<<std::endl;	
		};

                transit<Rotation>( action );
	}

	void react ( const GoalPose& e ) override 
	{	
		auto action = [=]() {
                        std::cout<<" send Goal Pose !"<<std::endl;

			int ret = udp_client_.write( e.goal_pose, TargetPoseMsgHeader );
			std::cout<<" send required goal pose : "<<ret<<std::endl;
                };

                transit<Navigation>( action );
	}

	void react ( const GoalYaw& e ) override
	{
		auto action = [=]() {
                        std::cout<<" send Goal Yaw !"<<std::endl;

			int ret = udp_client_.write( e.goal_yaw, TargetYawMsgHeader );
			std::cout<<" send required goal yaw : "<<ret<<std::endl;
                };

                transit<Rotation>( action );
	}
};

// ---------------------- State : Rotation --------------------- //
class Rotation : public RobotMotion
{
public:
	void entry( void ) override
        {
                std::cout<<"Rotation State !"<<std::endl;
        }

        void react( const ArrivedGoalYaw& e ) override
        {
                std::cout<<" arrived the goal yaw "<<std::endl;

                transit<Stop>();
        }

        void react( const TimeOut& e ) override
        {
                std::cout<<"goal yaw : time out "<<std::endl;
	
		auto action = [=]() {
                	int command = 0x04;
                        int ret = udp_client_.write( command, MoveCommanderHeader );
                        std::cout<<" send Stop Command : "<<ret<<std::endl;
		};

                transit<Stop>( action );
        }

	void react( const RelocalizedPoseDetected& e ) override
	{
		std::cout<<"got the relocalized pose "<<std::endl;

		auto action = [=]() {
			int command = 0x04;
                        int ret = udp_client_.write( command, MoveCommanderHeader );
                        std::cout<<" send Stop Command : "<<ret<<std::endl;
		};

		transit<Stop>( action );
	}

	void react( const ObstacleDetected& e ) override
	{
		std::cout<<"detected a obstacle "<<std::endl;

		transit<Stop>();
	}
};

// --------------------- State : Navigation -------------------- //
class Navigation : public RobotMotion
{
public:
        void entry( void ) override
        {
                std::cout<<"Naviation State !"<<std::endl;
        }

        void react( const ArrivedGoalPose& e ) override
        {
                std::cout<<" arrived the goal pose "<<std::endl;

                transit<Stop>();
        }


        void react( const TimeOut& e ) override
        {
                std::cout<<" goal yaw : time out "<<std::endl;

                transit<Stop>();
        }

	void react( const ObstacleDetected& e ) override
        {
                std::cout<<"detected a obstacle "<<std::endl;

                transit<Stop>();
        }
};


// ----------------------- State : Back ---------------------- //
class Back : public RobotMotion
{
public:

};

// ------------ Base State : default implementation ---------- //
void RobotMotion::react( const ArrivedGoalPose& e )
{

}

void RobotMotion::react( const ArrivedGoalYaw& e )
{

}

void RobotMotion::react( const TimeOut& e )
{

}

void RobotMotion::react ( const GoalPose& e )
{

}

void RobotMotion::react ( const GoalYaw& e )
{

}

void RobotMotion::react( const RelocalizationGoalPose& e )
{

}

void RobotMotion::react( const RelocalizationGoalYaw& e )
{

}

void RobotMotion::react( const RelocalizedPoseDetected& e )
{

}

void RobotMotion::react( const ObstacleDetected& e )
{

}

void RobotMotion::entry( void )
{

}

void RobotMotion::exit( void )
{

}

// ---------------- Initial State Definition ----------------- //
FSM_INITIAL_STATE( RobotMotion, Stop );

