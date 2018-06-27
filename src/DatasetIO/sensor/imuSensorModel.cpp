#include <cereal/archives/json.hpp>
#include <core/sensor/imuSensorModel.h>
#include <fstream>

namespace VGUGV
{
namespace Common
{

ImuSensorModel::ImuSensorModel(void)
    : mAccelerometerScaleFactor(1.0)
	, mAccelerometerSaturation(0.0)
    , mAccelerometerNoiseDensity(0.0)
    , mAccelerometerDriftNoiseDensity(0.0)
	, mGyroscopeScaleFactor(1.0)
    , mGyroscopeSaturation(0.0)
    , mGyroscopeNoiseDensity(0.0)
    , mGyroscopeDriftNoiseDensity(0.0)
    , mEarthAcceleration(9.781)
{

}

double
ImuSensorModel::getAccelerometerScaleFactor(void) const
{
    return mAccelerometerScaleFactor;
}

double
ImuSensorModel::getAccelerometerSaturation(void) const
{
    return mAccelerometerSaturation;
}

double
ImuSensorModel::getAccelerometerNoiseDensity(void) const
{
    return mAccelerometerNoiseDensity;
}

double
ImuSensorModel::getAccelerometerDriftNoiseDensity(void) const
{
    return mAccelerometerDriftNoiseDensity;
}

double
ImuSensorModel::getGyroscopeScaleFactor(void) const
{
    return mGyroscopeScaleFactor;
}

double
ImuSensorModel::getGyroscopeSaturation(void) const
{
    return mGyroscopeSaturation;
}

double
ImuSensorModel::getGyroscopeNoiseDensity(void) const
{
    return mGyroscopeNoiseDensity;
}

double
ImuSensorModel::getGyroscopeDriftNoiseDensity(void) const
{
    return mGyroscopeDriftNoiseDensity;
}

double
ImuSensorModel::getEarthAcceleration(void) const
{
    return mEarthAcceleration;
}

void
ImuSensorModel::setAccelerometerScaleFactor(double scaleFactor)
{
	mAccelerometerScaleFactor = scaleFactor;
}

void
ImuSensorModel::setAccelerometerSaturation(double saturation)
{
    mAccelerometerSaturation = saturation;
}

void
ImuSensorModel::setAccelerometerNoiseDensity(double noiseDensity)
{
    mAccelerometerNoiseDensity = noiseDensity;
}

void
ImuSensorModel::setAccelerometerDriftNoiseDensity(double driftNoiseDensity)
{
    mAccelerometerDriftNoiseDensity = driftNoiseDensity;
}

void
ImuSensorModel::setGyroscopeScaleFactor(double scaleFactor)
{
	mGyroscopeScaleFactor = scaleFactor;
}

void
ImuSensorModel::setGyroscopeSaturation(double saturation)
{
    mGyroscopeSaturation = saturation;
}

void
ImuSensorModel::setGyroscopeNoiseDensity(double noiseDensity)
{
    mGyroscopeNoiseDensity = noiseDensity;
}

void
ImuSensorModel::setGyroscopeDriftNoiseDensity(double driftNoiseDensity)
{
    mGyroscopeDriftNoiseDensity = driftNoiseDensity;
}

void
ImuSensorModel::setEarthAcceleration(double earthAcceleration)
{
    mEarthAcceleration = earthAcceleration;
}

} // namespace Common
} // namespace VGUGV
