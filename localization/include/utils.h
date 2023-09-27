#ifndef __UTILS_H
#define __UTILS_H

#include <Eigen/Dense>
#include <cmath>
#include <type_traits>

class Utils
{
public:
	template<typename T>
	static 
	typename std::enable_if_t<std::is_same_v<T, float> | std::is_same_v<T, double>>
	angleNormalize( T&& angle )
	{
		if( angle >= M_PI ) {
                        angle -= 2 * M_PI;
                }

                if( angle <= -M_PI ){
                        angle += 2 * M_PI;
                }
	}
};

#endif
