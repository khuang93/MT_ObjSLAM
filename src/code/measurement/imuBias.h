#pragma once
/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ImuBias.h
 * @date  Feb 2, 2012
 * @author Vadim Indelman, Stephen Williams
 */

#include <memory>
#include "core/common/builtin_types.h"

namespace VGUGV {
namespace Common {

class ImuBias {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

public:
	ImuBias() :
		mBiasAcc(0.0, 0.0, 0.0),
		mBiasGyro(0.0, 0.0, 0.0) {}

	ImuBias(const Vector3d& biasAcc, const Vector3d& biasGyro) :
		mBiasAcc(biasAcc),
		mBiasGyro(biasGyro) {}

	explicit ImuBias(const Vector6d& v) :
				mBiasAcc(v.head<3>()),
				mBiasGyro(v.tail<3>()) {}

public:
	/** return the accelerometer and gyro biases in a single vector */
	Vector6d vector() const {
		Vector6d v;
		v << mBiasAcc, mBiasGyro;
		return v;
	}

	/** get accelerometer bias */
	const Vector3d& accelerometer() const {
		return mBiasAcc;
	}

	/** get gyroscope bias */
	const Vector3d& gyroscope() const {
		return mBiasGyro;
	}

public:
	/** Correct an accelerometer measurement using this bias model, and optionally compute Jacobians */
	Vector3d correctAccelerometer(const Vector3d& measurement) const {
		return measurement - mBiasAcc;
	}

	/** Correct a gyroscope measurement using this bias model, and optionally compute Jacobians */
	Vector3d correctGyroscope(const Vector3d& measurement) const {
		return measurement - mBiasGyro;
	}

private:
	Vector3d mBiasAcc;
	Vector3d mBiasGyro;
}; // ImuBias class

} // namespace Common
} // namespace VGUGV

