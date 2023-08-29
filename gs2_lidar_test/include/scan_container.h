#ifndef __SCAN_CONTAINER_H
#define __SCAN_CONTAINER_H

#include <vector>
#include <cmath>
#include <type_traits>

#include <Eigen/Dense>

namespace sensor
{

template<typename T, int Rows, int Cols,
	 template<typename U, int R, int C, int Option, int MaxR, int MaxC>
	 class EigenType>
struct is_eigen_type : std::false_type
{
	
};

template<typename T, int Rows, int Cols>
struct is_eigen_type<T, Rows, Cols, Eigen::Matrix> : std::true_type
{
	using type = Eigen::Matrix<T, Rows, Cols>;
};

template<typename T, 
	 typename = typename std::enable_if_t<is_eigen_type<typename T::value_type, T::RowsAtCompileTime, T::ColsAtCompileTime, Eigen::Matrix>::value>>
class ScanContainer_
{
public:
	using type = T;
	using value_type = typename T::value_type;

	ScanContainer_() 
	{
	
	}

	~ScanContainer_()
	{
		
	}

	void add( const type& data )
	{
		data_vec_.push_back( data );
	}

	void clear()
	{
		return data_vec_.clear();
	}

	const type& at( const int idx ) const
	{
		return data_vec_[idx];
	}

	const int size() const
	{
		return data_vec_.size();
	}

	bool isEmpty() const
	{
		return data_vec_.empty();
	}

	type& operator[]( const int idx )
	{
		return data_vec_[idx];
	}

private:
	std::vector<type> data_vec_;
};

template<typename T>
using ScanContainer = ScanContainer_<typename Eigen::Matrix<T, 2, 1>>;

using ScanContainerI = ScanContainer<int>;
using ScanContainerF = ScanContainer<float>;
using ScanContainerD = ScanContainer<double>;

}

#endif
