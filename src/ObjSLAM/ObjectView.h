//
// Created by khuang on 6/6/18.
//

#ifndef MT_OBJSLAM_OBJECTVIEW_H
#define MT_OBJSLAM_OBJECTVIEW_H
#include <src/ObjSLAM/ObjSLAMDataTypes.h>
#include "../../External/InfiniTAM/InfiniTAM/ITMLib/Objects/Views/ITMView.h"
#include "../../External/InfiniTAM/InfiniTAM/ITMLib/Objects/Camera/ITMRGBDCalib.h"
#include "ObjCameraPose.h"
#include "ObjSLAMDataTypes.h"

namespace ObjSLAM {

 class ObjectView : public ITMLib::ITMView{

   //TODO: Finish all private members of ITMView with their correct data types
  private:
   ObjCameraPose *camera_Pose;
   
   ObjUIntImage *segmentation_Mask;
   ObjUChar4Image *rgb_Image;
   ObjFloatImage *depth_Image;



  public:

   //Constructor
   //using ITMLib::ITMView::ITMView;
   ObjectView(const ITMLib::ITMRGBDCalib& calibration, Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU, ObjCameraPose pose):
       ITMView(  calibration,  imgSize_rgb,  imgSize_d,  useGPU), camera_Pose(&pose){

   }

  //Destructor
   ~ObjectView(){
     delete camera_Pose;
     delete rgb_Image;
     delete depth_Image;
     delete segmentation_Mask;
//     ITMLib::ITMView::~ITMView();
   }


   ObjCameraPose* getCameraPose(){
     return camera_Pose;
   }


   void setCameraPose(ObjCameraPose *_pose){
     camera_Pose = _pose;
   }

   ObjectView(const ObjectView&);
   ObjectView& operator=(const ObjectView&);
  };

}
#endif //MT_OBJSLAM_OBJECTVIEW_H
