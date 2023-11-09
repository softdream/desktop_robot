#ifndef __GOAL_GENERATE_H
#define __GOAL_GENERATE_H

#include <iostream>
#include <random>
#include <Eigen/Dense>
#include <unistd.h>

namespace planning
{

template<typename T>
class Goal
{
public:
	using value_type = T;
	using Vector2 = Eigen::Matrix<value_type, 2, 1>;
	

	Goal()
	{
	
	}

	~Goal()
	{
	
	}
	
	void setEFence( const Vector2& e_fence_lt, const Vector2& e_fence_rb )
	{
		e_fence_lt_ = e_fence_lt;
		e_fence_rb_ = e_fence_rb;
	}

	const Vector2 generateTargetPose()
	{
		Vector2 planned_target_pose = Vector2::Zero();

		while ( 1 ) {
			randomPoseGenerate( planned_target_pose );

			if ( ( planned_target_pose - previous_pose_ ).norm() > 0.1 ) {
				previous_pose_ = planned_target_pose;

				return planned_target_pose;
			}	
			usleep( 100000 );
		}
	}

private:
	void randomPoseGenerate( Vector2& planned_target_pose )	
	{
		std::random_device rd;
		std::mt19937 gen;

		gen.seed( rd() );

		std::uniform_real_distribution<value_type> x( e_fence_rb_[0], e_fence_lt_[0] );
		std::uniform_real_distribution<value_type> y( e_fence_lt_[1], e_fence_rb_[1] );
		
		planned_target_pose[0] = x( gen );
		planned_target_pose[1] = y( gen );
	}

	void relocalizationPoseYawGenerate( Vector2& relocalization_target_pose, 
		       			    Vector2& relocalization_target_yaw )
	{
		relocalization_target_pose = Vector2( 3.0, 0.0 );
		relocalization_target_yaw = M_PI;
	}

private:
	Vector2 e_fence_lt_ = Vector2( 0.3, -0.3 );
	Vector2 e_fence_rb_ = Vector2( 0.05, 0.3 );
	
	Vector2 previous_pose_ = Vector2::Zero();
};

}

#endif
