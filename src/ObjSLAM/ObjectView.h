//
// Created by khuang on 6/6/18.
//

#ifndef MT_OBJSLAM_OBJECTVIEW_H
#define MT_OBJSLAM_OBJECTVIEW_H
#include "../../External/InfiniTAM/InfiniTAM/ITMLib/Objects/Views/ITMView.h"
#include "../../External/InfiniTAM/InfiniTAM/ITMLib/Objects/Camera/ITMRGBDCalib.h"
#include "ObjCameraPose.h"

namespace ObjSLAM {

 class ObjectView : public ITMLib::ITMView{

   //TODO: Finish all private members of ITMView with their correct data types
  private:
   ObjCameraPose *cameraPose;


  public:

   //Constructor
   //using ITMLib::ITMView::ITMView;
   ObjectView(const ITMLib::ITMRGBDCalib& calibration, Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU, ObjCameraPose pose):
       ITMView(  calibration,  imgSize_rgb,  imgSize_d,  useGPU), cameraPose(&pose){

   }

  //Destructor
   ~ObjectView(){
     delete cameraPose;
//     ITMLib::ITMView::~ITMView();
   }


   ObjCameraPose* getCameraPose(){
     return cameraPose;
   }


   void setCameraPose(ObjCameraPose *_pose){
     cameraPose = _pose;
   }

   ObjectView(const ObjectView&);
   ObjectView& operator=(const ObjectView&);
  };

}
#endif //MT_OBJSLAM_OBJECTVIEW_H
