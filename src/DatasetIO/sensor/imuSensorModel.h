#ifndef SRC_COMMON_SENSOR_IMUSENSORMODEL_H_
#define SRC_COMMON_SENSOR_IMUSENSORMODEL_H_

#include "sensorBase.h"
#include <cereal/cereal.hpp>

namespace VGUGV
{
namespace Common
{

class ImuSensorModel : public SensorBase
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    explicit ImuSensorModel(void);

    double getAccelerometerScaleFactor(void) const;
    double getAccelerometerSaturation(void) const;
    double getAccelerometerNoiseDensity(void) const;
    double getAccelerometerDriftNoiseDensity(void) const;
    double getGyroscopeScaleFactor(void) const;
    double getGyroscopeSaturation(void) const;
    double getGyroscopeNoiseDensity(void) const;
    double getGyroscopeDriftNoiseDensity(void) const;
    double getEarthAcceleration(void) const;

    void setAccelerometerScaleFactor(double scaleFactor);
    void setAccelerometerSaturation(double saturation);
    void setAccelerometerNoiseDensity(double noiseDensity);
    void setAccelerometerDriftNoiseDensity(double driftNoiseDensity);
    void setGyroscopeScaleFactor(double scaleFactor);
    void setGyroscopeSaturation(double saturation);
    void setGyroscopeNoiseDensity(double noiseDensity);
    void setGyroscopeDriftNoiseDensity(double driftNoiseDensity);
    void setEarthAcceleration(double earthAcceleration);
    
private:
    double mAccelerometerScaleFactor;
    double mAccelerometerSaturation;        ///< m/s^2
    double mAccelerometerNoiseDensity;      ///< m/s^2/sqrt(Hz)
    double mAccelerometerDriftNoiseDensity; ///< m/s^3/sqrt(Hz)

    double mGyroscopeScaleFactor;
    double mGyroscopeSaturation;   ///< rad/s
    double mGyroscopeNoiseDensity; ///< rad/s/sqrt(Hz)
    double mGyroscopeDriftNoiseDensity; ///< rad/s^2/sqrt(Hz)

    double mEarthAcceleration; ///< m/s^2
}; // ImuSensor

} // namespace Common
} // namespace VGUGV

#endif
