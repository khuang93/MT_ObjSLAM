#include "sensorBase.h"

namespace VGUGV
{
    namespace Common
    {
        void SensorBase::setDeviceID(int ID)
        {
            mDeviceID = ID;
        }

        void SensorBase::setDeviceName(const std::string &devName)
        {
            mDeviceName = devName;
        }

        void SensorBase::setT_Body2Sensor(const Transformation &T)
        {
            mT_Body2Sensor = T;
            computeJacobians();
        }

        void SensorBase::setT_Body2Sensor(const Eigen::Matrix4d &T)
        {
            mT_Body2Sensor = Sophus::SE3d(T);
            computeJacobians();
        }

        void SensorBase::setT_Body2Sensor(const Eigen::Matrix4d &T, const Eigen::Matrix<double, 6, 6> &covariance)
        {
            mT_Body2Sensor = Sophus::SE3d(T);
            computeJacobians();
        }

        int SensorBase::getDeviceID() const
        {
            return mDeviceID;
        }

        std::string SensorBase::getDeviceName() const
        {
            return mDeviceName;
        }

        const Transformation &SensorBase::getT_Body2Sensor() const
        {
            return mT_Body2Sensor;
        }

        Matrix4x4d SensorBase::getJacobian_t0() const
        {
            return mJacobianT_t0;
        }

        Matrix4x4d SensorBase::getJacobian_t1() const
        {
            return mJacobianT_t1;
        }

        Matrix4x4d SensorBase::getJacobian_t2() const
        {
            return mJacobianT_t2;
        }

        Matrix4x4d SensorBase::getJacobian_w0() const
        {
            return mJacobianT_w0;
        }

        Matrix4x4d SensorBase::getJacobian_w1() const
        {
            return mJacobianT_w1;
        }

        Matrix4x4d SensorBase::getJacobian_w2() const
        {
            return mJacobianT_w2;
        }

        void SensorBase::computeJacobians()
        {
            // jacobians of T_b^s * delta_T * T_s^b to se3 of delta_T
            // compute jacobians
            Matrix4x4d T_b2s = mT_Body2Sensor.matrix();
            Matrix4x4d T_s2b = mT_Body2Sensor.inverse().matrix();

            mJacobianT_t0 = T_b2s * SE3d::generator(0) * T_s2b;
            mJacobianT_t1 = T_b2s * SE3d::generator(1) * T_s2b;
            mJacobianT_t2 = T_b2s * SE3d::generator(2) * T_s2b;

            mJacobianT_w0 = T_b2s * SE3d::generator(3) * T_s2b;
            mJacobianT_w1 = T_b2s * SE3d::generator(4) * T_s2b;
            mJacobianT_w2 = T_b2s * SE3d::generator(5) * T_s2b;
        }
    } //namespace Common
} // namespace VGUGV