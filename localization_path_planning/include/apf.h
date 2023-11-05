#ifndef __APF_H
#define __APF_H

#include "data_type.h"
#include <vector>
#include <cmath>

namespace planning
{

template<typename T>
class APF
{
public:
	using value_type = T;
	using Vector2 = typename Eigen::Matrix<T, 2, 1>;

	APF()
	{
	
	}

	~APF()
	{
	
	}

	// target pose
	void setTargetPose( const Vector2& target_pose )
	{
		target_pose_ = target_pose;
	}

	// attraction
	const Vector2 cacuFatt( const Vector2& robot_pose )
	{
		value_type dist = distance( robot_pose, target_pose_ );
		value_type f_att_norm = dist * eta_;

		Vector2 direction = ( target_pose_ - robot_pose ).normalized();

		return f_att_norm * direction;
	}

	// repulsion
	const Vector2 cacuFrep( const Vector2& robot_pose, 
		       		const std::vector<sensor::Obstacle<value_type>>& obstacles )
	{
		if ( obstacles.empty() ) return Vector2::Zero();

		value_type rho_g = distance( robot_pose, target_pose_ );

		// for each obstacle
		Vector2 f_rep_total = Vector2::Zero();
		for ( const auto& obs : obstacles ) {
			value_type dist = distance( robot_pose, obs.pose );

			value_type f_rep1_norm = 0.0;
			value_type f_rep2_norm = 0.0;

			if ( dist < rho0_ ) {
				f_rep1_norm = cauchy_ * ( 1 / dist - 1 / rho0_ ) * ( ::pow( rho_g, n_ ) ) / ( ::pow( dist, 2 ) );
				f_rep2_norm = ( static_cast<value_type>( n_ ) * cauchy_ * 0.5 ) * ( ::pow( ( 1 / dist - 1 / rho0_ ), 2 ) ) * ( ::pow( rho_g, n_ - 1 ) );
			}

			Vector2 f_rep1_direction = ( robot_pose - obs.pose ).normalized();
			Vector2 f_rep2_direction = ( target_pose_ - robot_pose ).normalized();

			f_rep_total += ( f_rep1_norm * f_rep1_direction + f_rep2_norm * f_rep2_direction );
		}

		return f_rep_total;
	}

private:
	const value_type distance( const Vector2& p1, const Vector2& p2 ) const
	{
		return ( p1 - p2 ).norm();
	}

private:
	// 1. apf parameters
	constexpr static value_type eta_ = 0.5f;
	constexpr static value_type rho0_ = 0.15;
	constexpr static value_type cauchy_ = 0.005;
	constexpr static int n_ = 3;

	Vector2 target_pose_ = Vector2::Zero();
};

}

#endif
