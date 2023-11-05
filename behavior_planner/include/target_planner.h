#ifndef __TARGET_PLANNER_H
#define __TARGET_PLANNER_H

#include <iostream>
#include <random>
#include <Eigen/Dense>

namespace planning

template<typename T>
class TargetPlanner
{
public:
	using value_type = T;
	using Vector2 = Eigen::Matrix<value_type, 2, 1>;
	

	TargetPlanner()
	{
	
	}

	~TargetPlanner()
	{
	
	}
	
	void setEFence( const Vector2& e_fence_lt, const Vector2& e_fence_rb )
	{
		e_fence_lt_ = e_fence_lt;
		e_fence_rb_ = e_fence_rb;
	}


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
	Vector2 e_fence_lt_( 0.3, -0.3 );
	Vector2 e_fence_rb_( 0.0, 0.3 );
	
};


#endif
