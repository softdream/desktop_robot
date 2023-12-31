#ifndef __EKF_FUSION_H
#define __EKF_FUSION_H

#include <iostream>

#include "utils.h"

#ifndef M_PI 
#define M_PI 3.141592653
#endif

namespace ekf
{

template<typename T>
class EKF
{
public:
	using DataType = T;
	using Vector2 = typename Eigen::Matrix<T, 2, 1>;
	using Vector3 = typename Eigen::Matrix<T, 3, 1>;
	using Matrix2 = typename Eigen::Matrix<T, 2, 2>;
	using Matrix3 = typename Eigen::Matrix<T, 3, 3>;
	using Matrix1x3 = typename Eigen::Matrix<T, 1, 3>;
	using Matrix3x1 = typename Eigen::Matrix<T, 3, 1>;

	EKF()
	{

	}

	~EKF()
	{

	}

	// input control vector: ( delta_s, delta_theta )
	void predict( const Vector2& u_now ) 
	{
#ifdef DEBUG
		std::cout<<" start predict "<<std::endl;
#endif
		// 1. state prediction
		x_now_(0) = x_pre_(0) + u_now(0) * ::cos( x_pre_(2) + 0.5 * u_now(1) ); // x
		x_now_(1) = x_pre_(1) + u_now(0) * ::sin( x_pre_(2) + 0.5 * u_now(1) ); // y
		x_now_(2) = x_pre_(2) + u_now(1); // theta

		// added
		//Utils::angleNormalize( x_now_[2] );

		// 2. caculate state Jacobian matrix	
		F_ = Matrix3::Identity();
		F_(0, 2) = -u_now(0) * ::sin( x_pre_(2) + 0.5 * u_now(1) );
		F_(1, 2) =  u_now(0) * ::cos( x_pre_(2) + 0.5 * u_now(1) );
	
		// 3. state covarince prediction
		P_now_ = F_ * P_pre_ * F_.transpose() + Q_;
	
		// testing
		//x_pre_ = x_now_;
                //P_pre_ = P_now_;
	}	

	void update( const DataType z ) // measurement value : delta_theta
	{

#ifdef DEBUG
        	std::cout<<" update "<<std::endl;
#endif

		// 1. measurement estimate
		H_ << 0, 0, 1;
		DataType h = H_ * x_now_ - x_pre_(2);
		
		// 2. measurement error
		DataType error = z - h;
	
		// 3. Kalman Gain
		DataType K_tmp = H_ * P_now_ * H_.transpose() + R_;
		Matrix3x1 K = P_now_ * H_.transpose() * ( 1 / K_tmp );

		// 4. state update
		x_now_ += K * error;
		
		// 5. state covarince matrix update
		P_now_ = ( Matrix3::Identity() - K * H_ ) * P_now_;

		// 6. angle normalize
		Utils::angleNormalize( x_now_[2] );
	
		// 8. update the old value
                x_pre_ = x_now_;
                P_pre_ = P_now_;
	}
 
	void update( const Vector3& z )
	{
	
		// 1. measurement estimate
		auto h = H_cam_ * x_now_;

		// 2. measurement error
		auto error = z - h;

		// 3. Kalman Gain
		auto K_tmp = H_cam_ * P_now_ * H_cam_.transpose() + R_cam_;
		Matrix3 K = P_now_ * H_cam_.transpose() * ( K_tmp.inverse() );

		// 4. state update
		x_now_ += K * error;

		// 5. state covarince matrix update
		P_now_ = ( Matrix3::Identity() - K * H_cam_ ) * P_now_();

		// 6. update the old value
		x_pre_ = x_now_;
		P_pre_ = P_now_;
	}


	const Vector3& getStateX() const
	{
		return x_now_;
	}

	void setStateGaussianNoise( const Matrix1x3& H )
	{
		H_ = H;
	}

	void setMeasurementGaussianNoise( const DataType R )
	{
		R_ = R;
	}

	void resetState( const Vector3& x )
	{
		x_pre_ = x;
		x_now_ = x;	

		// added
		P_pre_ = Matrix3::Identity();
		P_now_ = Matrix3::Identity();
	}

	const Matrix3& getCovarinceMatrixP()
	{
		return P_now_;
	}
	
	
private:
	// state vector at k-1 moment, ( x, y, theta )
	Vector3 x_pre_ = Vector3::Zero();
	
	// state vector at k moment, ( x, y, theta )
	Vector3 x_now_ = Vector3::Zero();

	// state covarince matrix
	Matrix3 P_pre_ = Matrix3::Identity();
	Matrix3 P_now_ = Matrix3::Identity();

	// state Jacobian matrix
	Matrix3 F_ = Matrix3::Identity();

	// measurement update matrix
	Matrix1x3 H_ = Matrix1x3::Zero();

	// added : 
	Matrix3 H_cam_ = Matrix3::Identity();

	// state Gaussian Noise
	Matrix3 Q_ = Matrix3::Identity() * 1000;

	// measurement Gaussian Noise
	DataType R_ = 10;
	
	// added : measurement Gaussian Noise by camera aruco detection
	Matrix3 R_cam_ = Matrix3::Identity();

};

}

#endif
