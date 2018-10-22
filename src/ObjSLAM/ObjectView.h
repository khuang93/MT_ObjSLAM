//
// Created by khuang on 8/1/18.
//
#pragma once
#ifndef MT_OBJSLAM_OBJECTVIEW_NEW_H
#define MT_OBJSLAM_OBJECTVIEW_NEW_H

#include <vector>
#include <memory>

#include "src/ObjSLAM/ObjSLAMDataTypes.h"

#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Views/ITMView.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Camera/ITMRGBDCalib.h"
#include "src/ObjSLAM/ObjCameraPose.h"
#include "ObjSLAMDataTypes.h"

#include <utility>
#include <map>

#include "External/InfiniTAM/InfiniTAM/ORUtils/MemoryBlock.h"
#include "External/InfiniTAM/InfiniTAM/ORUtils/Image.h"

namespace ObjSLAM {
    template<typename TVoxel, typename TIndex>
    class ObjectInstance;

    template<typename TVoxel, typename TIndex>
    class ObjectClassLabel_Group;

    template<typename TVoxel, typename TIndex>
    using Object_View_Tup = std::tuple<std::shared_ptr<ObjectInstance<TVoxel, TIndex>>,
            std::shared_ptr<ITMLib::ITMView>>;


    template<typename TVoxel, typename TIndex>
    class ObjectView : public enable_shared_from_this<ObjectView<TVoxel, TIndex>> {
    private:
        bool is_RGB_D_aligned = false;

        ObjCameraPose *camera_Pose;

        std::shared_ptr<ObjUIntImage> segmentation_Mask;
        LabelImgVector label_img_vector;
        /*const*/ ObjUChar4Image *rgb_Image;
        /*const*/ ObjFloatImage *depth_Image;
//  ObjFloat4Image *depth_normal;

        const ITMLib::ITMRGBDCalib &calibration;
        Vector2i imgSize_rgb;
        Vector2i imgSize_d;

        std::vector<Object_View_Tup<TVoxel, TIndex>> obj_view_tup_vec;
        std::shared_ptr<ITMLib::ITMView> bg_itmview;

        std::vector<int> rgb_d_pixel_idx_vec; //index is index in rgb img and value is index in depth img
        shared_ptr<ORUtils::Image<Vector2i>> d_to_rgb_correspondence; //stored the pixel location in rgb img at corresponding depth pixel location

//  std::vector<ITMLib::ITMView*> ITMViewVector_each_Object;


//  std::map<int, Object_View_Tup<TVoxel,TIndex>> obj_map; //int is the raw value in seg mask and tuple contains a obj instance and corresbonding ITMView



        void setListOfViews();

    public:

        //Constructor simple
        ObjectView(const ITMLib::ITMRGBDCalib &_calibration, Vector2i _imgSize, bool useGPU, ObjCameraPose pose,
                   vector<int> _rgb_d_pixel_idx_vec) :
                calibration(_calibration), imgSize_rgb(_imgSize), rgb_d_pixel_idx_vec(_rgb_d_pixel_idx_vec) {
            camera_Pose = new ObjCameraPose(pose.GetSE3Pose());
        }

        //Constructor with pose
        ObjectView(const ITMLib::ITMRGBDCalib &_calibration,
                   Vector2i _imgSize_rgb,
                   Vector2i _imgSize_d,
                   bool useGPU,
                   ObjCameraPose pose,
                   ObjFloatImage *_depth,
                   ObjUChar4Image *_rgb,
                   LabelImgVector _label_img_vector,
                   vector<int> _rgb_d_pixel_idx_vec) :
                calibration(_calibration), imgSize_rgb(_imgSize_rgb), imgSize_d(_imgSize_d), /*camera_Pose(pose),*/
                /*depth_Image(_depth), rgb_Image(_rgb),*/ label_img_vector(_label_img_vector),
                rgb_d_pixel_idx_vec(_rgb_d_pixel_idx_vec) {

            camera_Pose = new ObjCameraPose(pose.GetSE3Pose());
            rgb_Image->SetFrom(_rgb, ORUtils::Image<Vector4u>::CPU_TO_CPU);
            depth_Image->SetFrom(_depth, ORUtils::Image<float>::CPU_TO_CPU);
            //TODO debug info
//    std::cout<<"ObjectView complete created!\n";

        }

        //Constructor, currently using this
        ObjectView(const ITMLib::ITMRGBDCalib &_calibration,
                   Vector2i _imgSize_rgb,
                   Vector2i _imgSize_d,
                   bool useGPU,
                   ObjFloatImage *_depth,
                   ObjUChar4Image *_rgb,
                   LabelImgVector _label_img_vector) :
                calibration(_calibration), imgSize_rgb(_imgSize_rgb), imgSize_d(_imgSize_d),
                depth_Image(_depth), rgb_Image(_rgb), label_img_vector(_label_img_vector) {
                    if(calibration.trafo_rgb_to_depth.calib==calibration.trafo_rgb_to_depth.calib_inv){
                        is_RGB_D_aligned=true;
                    }
        }

        //Destructor
        ~ObjectView() {
            delete rgb_Image;
            delete depth_Image;
            delete camera_Pose;
        }

        void SetListOfObjects(std::vector<shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>>> &label_ptr_vector);

        void SetListOfObjects_New(std::vector<shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>>> &label_ptr_vector);

        void
        UpdateObjectTrackingState(std::vector<shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>>> &label_ptr_vector);


        static std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>> AddLabelToVector(
                std::vector<shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>>> &label_ptr_vector,
                std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>> new_label);


        ObjCameraPose GetCameraPose();

        void SetCameraPose(ObjCameraPose _pose);

        void SetCameraPose(const ORUtils::SE3Pose *_se3pose);

        std::vector<Object_View_Tup<TVoxel, TIndex>>& GetObjVec();

        std::shared_ptr<ITMLib::ITMView> GetBackgroundView();

        ObjectView(const ObjectView &);

        ObjectView &operator=(const ObjectView &);
    };

}

#include "ObjectView.tpp"

#endif //MT_OBJSLAM_OBJECTVIEW_NEW_H
