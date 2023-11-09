#ifndef __APF_PROCESS_H
#define __APF_PROCESS_H

#include "apf.h"

namespace planning
{

template<typename T>
class APFProcess
{
public:
	using value_type = T;
	using Vector2 = typename Eigen::Matrix<T, 2, 1>;

	APFProcess()
        {

        }

        ~APFProcess()
        {

        }

	void setTargetPose( const Vector2& target_pose )
        {
                return apf_.setTargetPose( target_pose );
        }

	const std::pair<value_type, value_type>
	runApfOnce( const Vector2& robot_pose, const std::vector<sensor::Obstacle<value_type>>& obstacles )
	{
		// 1. attraction vector
		Vector2 f_att_vec = apf_.cacuFatt( robot_pose );

		// 2. repulsion vector
		Vector2 f_rep_vec = apf_.cacuFrep( robot_pose, obstacles );

		// 3. total force
		Vector2 f_total = f_att_vec + f_rep_vec;

		// 4. get target angle
		value_type target_theta = ::atan2( f_total[1], f_total[0] );

		return { f_total.norm(), target_theta };
	}

private:
        APF<value_type> apf_;
};

}

#endif
