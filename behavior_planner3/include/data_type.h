#ifndef __DATA_TYPE_H
#define __DATA_TYPE_H

#include <stdint.h>
#include <Eigen/Dense>


namespace sensor
{

// -------------------------------- 1. IMU ------------------------------------ //
template<typename T>
struct ImuData_
{
	using value_type = T;

	ImuData_()
	{
	
	}

	ImuData_( const value_type acc_x_, const value_type acc_y_, const value_type acc_z_, const value_type gyro_x_, const value_type gyro_y_, const value_type gyro_z_ )
		: acc_x( acc_x_ ), acc_y( acc_y_ ), acc_z( acc_z_ ), gyro_x( gyro_x_ ), gyro_y( gyro_y_ ), gyro_z( gyro_z_ )
	{
	
	}

	value_type acc_x = 0;
	value_type acc_y = 0;
	value_type acc_z = 0;

	value_type gyro_x = 0;
	value_type gyro_y = 0;
	value_type gyro_z = 9;
};

template<typename T>
using ImuData = ImuData_<T>;

using ImuDataI = ImuData<int>;
using ImuDataF = ImuData<float>;
using ImuDataD = ImuData<double>;

// -------------------------------- 2. MOTOR ------------------------------------ //
template<typename T>
struct MotorData_
{
	using value_type = T;
	
	MotorData_()
	{
	
	}

	MotorData_( const value_type l_rpm_,
		    const value_type r_rpm_,
		    const value_type velocity_,
		    const value_type delta_s_,
		    const value_type delta_angle_ ) :
			l_rpm( l_rpm_ ),
			r_rpm( r_rpm_ ),
			velocity( velocity_ ),
			delta_s( delta_s_ ),
			delta_angle( delta_angle_ )
	{
	
	}

	value_type l_rpm = 0;
	value_type r_rpm = 0;
	value_type velocity = 0;
	value_type delta_s = 0;
	value_type delta_angle = 0;
};

template<typename T>
using MotorData = MotorData_<T>;

using MotorDataI = MotorData<int>;
using MotorDataF = MotorData<float>;
using MotorDataD = MotorData<double>;

// -------------------------------- 3. Pose 2D ------------------------------------ //
template<typename T>
struct Pose2D_
{
	using value_type = T;
	
	Pose2D_()
	{
	
	}
	
	~Pose2D_()
	{
	
	}

	Pose2D_( const value_type x_, const value_type y_, value_type theta_ )
		: x( x_ ), y( y_ ), theta( theta_ )
	{
	
	}

	value_type x = 0.0;
	value_type y = 0.0;
	value_type theta = 0.0;
};

template<typename T>
using Pose2D = Pose2D_<T>;

using Pose2DI = Pose2D<int>;
using Pose2DF = Pose2D<float>;
using Pose2DD = Pose2D<double>;

// -------------------------------- 4. Lidar Data ------------------------------------ //
template<typename T>
struct LaserPoint_
{
        using value_type = T;

        value_type angle = 0;
        value_type range = 0;
        value_type intensity = 0;
};

template<typename T>
using LaserPoint = LaserPoint_<T>;

using LaserPointI = LaserPoint<int>;
using LaserPointF = LaserPoint<float>;
using LaserPointD = LaserPoint<double>;

template<typename T, int Size>
struct LidarScan_
{
        using value_type = T;

        uint64_t stamp = 0;

        value_type min_angle = -0.87266;
        value_type max_angle = 0.87266;
        value_type angle_increment = 0.01090825;
        value_type min_range = 0.02;
        value_type max_range = 0.3;

        constexpr int size() const
        {
                return Size;
        }

        LaserPoint<value_type> points[Size] = {0};
};

template<typename T>
using LidarScan = LidarScan_<T, 150>;

using LidarScanI = LidarScan<int>;
using LidarScanF = LidarScan<float>;
using LidarScanD = LidarScan<double>;

// -------------------------------- 5. Obstacle ------------------------------------ //
template<typename T>
struct Obstacle_
{
	using value_type = T;
	using pose_type = Eigen::Matrix<value_type, 2, 1>;

	pose_type pose = pose_type::Zero();
	value_type size = 0;
};

template<typename T>
using Obstacle = Obstacle_<T>;

using ObstacleI = Obstacle<int>;
using ObstacleF = Obstacle<float>;
using ObstacleD = Obstacle<double>;

// ------------------------------- 6. Message Type ---------------------------------- //
enum MessageType
{
	ArriveGoalPose = 1, // arrived the goal pose
	ArriveGoalYaw, // arrived the goal yaw
	Timeout, // path tracking time out
	ReLocalization, // relocalization
	GotRelocalizedPose, // got the relocalized pose
	RelocalizaitonTimeOut, // relocalizaiton time out
	ObstacleDetected, // detected a obstacle
	IsKeyPose,  // key pose
};

}

// -------------------- Data transport UDP Port Definitions ------------------------ //
static constexpr int LocalizationProcessRecverPort = 2333;
static constexpr int BehaviorPlannerProcessRecverPort = 2334;
static constexpr int LandMarkerPoseProcessPort = 2335;

// --------------------------- PreDefined Messages Header -------------------------- //
static constexpr char TargetPoseMsgHeader = 0x01;
static constexpr char TargetYawMsgHeader = 0x02;
static constexpr char RelocalizationMsgHeader = 0x03;
static constexpr char MoveCommanderHeader = 0x04;


#endif
