//
// Created by khuang on 6/24/18.
//

#ifndef MT_OBJSLAM_LPD_RAW_POSE_H
#define MT_OBJSLAM_LPD_RAW_POSE_H
namespace ObjSLAM {
struct LPD_RAW_Pose {
/**
 * \brief The pose used in the LPD Dataset
 *
 */
  double t, qw, qx, qy, qz;
  double x, y, z, vx, vy, vz;
  double p, q, r, ax, ay, az;
};
}
#endif //MT_OBJSLAM_LPD_RAW_POSE_H
