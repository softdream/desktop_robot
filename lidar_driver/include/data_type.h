#ifndef __DATA_TYPE_H
#define __DATA_TYPE_H

namespace sensor{

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

}

#endif
