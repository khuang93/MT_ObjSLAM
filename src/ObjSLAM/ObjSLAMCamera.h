//
// Created by khuang on 7/26/18.
//

#ifndef MT_OBJSLAM_OBJSLAMCAMERA_H
#define MT_OBJSLAM_OBJSLAMCAMERA_H

#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/Camera/ITMRGBDCalib.h>
#include <External/InfiniTAM/InfiniTAM/ORUtils/SE3Pose.h>
#include "ObjSLAMDataTypes.h"
#include "External/InfiniTAM/InfiniTAM/ORUtils/Image.h"
#include "ObjCameraPose.h"

namespace ObjSLAM {
    class ObjSLAMCamera {
    private:
        const std::shared_ptr<ITMLib::ITMRGBDCalib> calib;
        int width, height;
        Vector2i imgSize;
        ORUtils::Matrix4<float> K_rgb;
        ORUtils::Matrix4<float> K_d;

    public:
        /**
         * @brief Constructor
         * @param _calib camera calibration
         * @param _imgSize image size
         */
        ObjSLAMCamera(const std::shared_ptr<ITMLib::ITMRGBDCalib> _calib, Vector2i _imgSize) : calib(_calib),
                                                                                               imgSize(_imgSize) {
            width = imgSize.width;
            height = imgSize.height;
            K_d = ORUtils::Matrix4<float>(this->calib->intrinsics_d.projectionParamsSimple.fx,
                                          0.0f,
                                          0.0f,
                                          0.0f,
                                          0.0f,
                                          this->calib->intrinsics_d.projectionParamsSimple.fy,
                                          0.0f,
                                          0.0f,
                                          this->calib->intrinsics_d.projectionParamsSimple.px,
                                          this->calib->intrinsics_d.projectionParamsSimple.py,
                                          1.0f,
                                          0.0f,
                                          0.0f,
                                          0.0f,
                                          0.0f,
                                          1.0f);
            K_rgb = ORUtils::Matrix4<float>(this->calib->intrinsics_rgb.projectionParamsSimple.fx,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            this->calib->intrinsics_rgb.projectionParamsSimple.fy,
                                            0.0f,
                                            0.0f,
                                            this->calib->intrinsics_rgb.projectionParamsSimple.px,
                                            this->calib->intrinsics_rgb.projectionParamsSimple.py,
                                            1.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            1.0f);
        }

/**
 * @brief Constructor
 * @param _calib camera calibration
 * @param w image width
 * @param h image height
 */
        ObjSLAMCamera(std::shared_ptr<ITMLib::ITMRGBDCalib> _calib, int w, int h) : calib(_calib), imgSize(w, h),
                                                                                    width(w), height(h) {}

    public:

        /**
         * @brief Using camera pose, project a point cloud in world coordinate into the camera
         * @param PCL Point cloud
         * @param out output image
         * @param pose camera pose
         */
        void ProjectPointCloud2Img(ORUtils::Image<Vector4f> *PCL, ObjFloatImage *out, ObjCameraPose pose);

        /**
         * @brief Using camera pose, project a depth image to point cloud in world coordinate
         * @param in input dept image
         * @param PCL output point cloud
         * @param pose camera pose
         * @return 6 corners of the bounding cube
         */
        ORUtils::Vector6<float>
        ProjectImg2PointCloud(ObjFloatImage *in, ORUtils::Image<Vector4f> *PCL, ObjCameraPose pose);

        shared_ptr<ORUtils::Image<Vector2i>> ProjectDepthPixelToRGB(ObjSLAM::ObjFloatImage *in);

//getters
        const std::shared_ptr<ITMLib::ITMRGBDCalib> GetCalib() { return calib; }

        Vector2i GetImgSize() { return imgSize; }

        int GetWidth() { return width; }

        int GetHeight() { return height; }

        ORUtils::Matrix4<float> getK_rgb() { return K_rgb; }

        ORUtils::Matrix4<float> getK_d() { return K_d; }
    };
}
#endif //MT_OBJSLAM_OBJSLAMCAMERA_H
