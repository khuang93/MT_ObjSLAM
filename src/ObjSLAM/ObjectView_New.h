//
// Created by khuang on 6/25/18.
//

#ifndef MT_OBJSLAM_OBJECTVIEW_NEW_H
#define MT_OBJSLAM_OBJECTVIEW_NEW_H


#include <vector>

#include "src/ObjSLAM/ObjSLAMDataTypes.h"

#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Views/ITMView.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Camera/ITMRGBDCalib.h"
#include "src/ObjSLAM/ObjCameraPose.h"
#include "ObjSLAMDataTypes.h"
#include "ObjectInstance.h"

#include <utility>
#include <map>

#include "External/InfiniTAM/InfiniTAM/ORUtils/MemoryBlock.h"
#include "External/InfiniTAM/InfiniTAM/ORUtils/Image.h"



namespace ObjSLAM {

using Object_View_Pair = std::pair<ObjectInstance*, ITMLib::ITMView*>;
class ObjectView_New {

 private:
  ObjCameraPose *camera_Pose;

  ObjUIntImage *segmentation_Mask;
  ObjUChar4Image *rgb_Image;
  ObjFloatImage *depth_Image;

  const ITMLib::ITMRGBDCalib& calibration;
  Vector2i imgSize;

  std::vector<ObjectInstance> objectInstanceVector;
  std::vector<ITMLib::ITMView*> ITMViewVector_each_Object;


  std::vector<Object_View_Pair> object_view_pair_vector;

  std::map<int, Object_View_Pair> obj_map;

  void setListOfObjects();
  void setListOfViews();


 public:

  //Constructor
  //using ITMLib::ITMView::ITMView;

  ObjectView_New(const ITMLib::ITMRGBDCalib& _calibration, Vector2i _imgSize, bool useGPU, ObjCameraPose pose):
      calibration(_calibration), imgSize(_imgSize), camera_Pose(&pose){

    setListOfObjects();



    //TODO debug info
    std::cout<<"ObjectView_New simple created!\n";
  }

  //Constructor
  //using ITMLib::ITMView::ITMView;
  ObjectView_New(const ITMLib::ITMRGBDCalib& _calibration, Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU, ObjCameraPose pose,
                 ObjFloatImage* _depth, ObjUChar4Image* _rgb, ObjUIntImage* _label):
      calibration(_calibration), camera_Pose(&pose), depth_Image(_depth), rgb_Image(_rgb), segmentation_Mask(_label){

    setListOfObjects();
    //TODO debug info
    std::cout<<"ObjectView_New complete created!\n";

  }

  //Destructor
  ~ObjectView_New(){
    delete camera_Pose;
    delete rgb_Image;
    delete depth_Image;
    delete segmentation_Mask;
//     ITMLib::ITMView::~ITMView();
  }


  ObjCameraPose* getCameraPose();
  std::map<int, Object_View_Pair> getObjMap();

  void setCameraPose(ObjCameraPose *_pose);



  ObjectView_New(const ObjectView_New&);
  ObjectView_New& operator=(const ObjectView_New&);
};

}
#endif //MT_OBJSLAM_OBJECTVIEW_NEW_H
