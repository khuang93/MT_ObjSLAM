//
// Created by khuang on 6/6/18.
//

#ifndef MT_OBJSLAM_OBJECTVIEW_H
#define MT_OBJSLAM_OBJECTVIEW_H
#include <vector>

#include "src/ObjSLAM/ObjSLAMDataTypes.h"

#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Views/ITMView.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Camera/ITMRGBDCalib.h"
#include "src/ObjSLAM/ObjCameraPose.h"
#include "ObjSLAMDataTypes.h"
#include "ObjectInstance.h"

#include "External/InfiniTAM/InfiniTAM/ORUtils/MemoryBlock.h"
#include "External/InfiniTAM/InfiniTAM/ORUtils/Image.h"

namespace ObjSLAM {

 class ObjectView : public ITMLib::ITMView{

   //TODO: Finish all private members of ITMView with their correct data types
  private:
   ObjCameraPose *camera_Pose;

   ObjUIntImage *segmentation_Mask;
   ObjUChar4Image *rgb_Image;
   ObjFloatImage *depth_Image;

   std::vector<ObjectInstance> objectInstanceVector;

   void setListOfObjects();


  public:

   //Constructor
   //using ITMLib::ITMView::ITMView;
   ObjectView(const ITMLib::ITMRGBDCalib& calibration, Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU, ObjCameraPose pose):
       ITMView(  calibration,  imgSize_rgb,  imgSize_d,  useGPU), camera_Pose(&pose){

     setListOfObjects();
     rgb=rgb_Image;
     depth=depth_Image;


     //TODO debug info
     std::cout<<"ObjectView simple ceated!\n";
   }

   //Constructor
   //using ITMLib::ITMView::ITMView;
   ObjectView(const ITMLib::ITMRGBDCalib& calibration, Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU, ObjCameraPose pose, ObjFloatImage* _depth, ObjUChar4Image* _rgb, ObjUIntImage* _label):
       ITMView(  calibration,  imgSize_rgb,  imgSize_d,  useGPU), camera_Pose(&pose), depth_Image(_depth), rgb_Image(_rgb), segmentation_Mask(_label){

     setListOfObjects();
     //TODO debug info
     std::cout<<"ObjectView complete ceated!\n";

   }

  //Destructor
   ~ObjectView(){
     delete camera_Pose;
     delete rgb_Image;
     delete depth_Image;
     delete segmentation_Mask;
//     ITMLib::ITMView::~ITMView();
   }


   ObjCameraPose* getCameraPose();

   void setCameraPose(ObjCameraPose *_pose);



   ObjectView(const ObjectView&);
   ObjectView& operator=(const ObjectView&);
  };

}
#endif //MT_OBJSLAM_OBJECTVIEW_H
