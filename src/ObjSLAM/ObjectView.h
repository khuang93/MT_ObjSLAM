//
// Created by khuang on 6/25/18.
//

#ifndef MT_OBJSLAM_OBJECTVIEW_H
#define MT_OBJSLAM_OBJECTVIEW_H


#include <vector>
#include <memory>

#include "src/ObjSLAM/ObjSLAMDataTypes.h"

#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Views/ITMView.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Camera/ITMRGBDCalib.h"
#include "src/ObjSLAM/ObjCameraPose.h"
#include "ObjSLAMDataTypes.h"
//#include "ObjectInstance.h"

#include <utility>
#include <map>

#include "External/InfiniTAM/InfiniTAM/ORUtils/MemoryBlock.h"
#include "External/InfiniTAM/InfiniTAM/ORUtils/Image.h"



namespace ObjSLAM {

class ObjectInstance;

using Object_View_Tuple = std::tuple<std::shared_ptr<ObjectInstance>, std::shared_ptr<ITMLib::ITMView>>;
using LabelImgVector = std::vector<std::shared_ptr<ObjSLAM::ObjUIntImage>>;



class ObjectView {

 private:
  ObjCameraPose camera_Pose;

  std::shared_ptr<ObjUIntImage> segmentation_Mask;
  LabelImgVector label_img_vector;
  const ObjUChar4Image *rgb_Image;
  const ObjFloatImage *depth_Image;
  ObjFloat4Image *depth_normal;

  const ITMLib::ITMRGBDCalib& calibration;
  Vector2i imgSize_rgb;
  Vector2i imgSize_d;


  std::vector<ObjectInstance> objectInstanceVector;
  std::vector<ITMLib::ITMView*> ITMViewVector_each_Object;


//  std::vector<Object_View_Tuple> object_view_pair_vector;

  std::map<int, Object_View_Tuple> obj_map; //int is the raw value in seg mask and tuple contains a obj instance and corresbonding ITMView

  void setListOfObjects_old();
  void setListOfObjects();
  void setListOfViews();


 public:

  //Constructor
  //using ITMLib::ITMView::ITMView;

  ObjectView(const ITMLib::ITMRGBDCalib& _calibration, Vector2i _imgSize, bool useGPU, ObjCameraPose pose):
      calibration(_calibration), imgSize_rgb(_imgSize), camera_Pose(pose){

    setListOfObjects_old();

    //TODO debug info
    std::cout<<"ObjectView simple created!\n";
  }

  //Constructor
  //using ITMLib::ITMView::ITMView;
/*  ObjectView(const ITMLib::ITMRGBDCalib& _calibration, Vector2i _imgSize_rgb, Vector2i _imgSize_d, bool useGPU, ObjCameraPose pose,
                 ObjFloatImage* _depth, ObjUChar4Image* _rgb, std::shared_ptr<ObjUIntImage> _label):
      calibration(_calibration), imgSize_rgb(_imgSize_rgb), imgSize_d(_imgSize_d), camera_Pose(pose),
      depth_Image(_depth), rgb_Image(_rgb), segmentation_Mask(_label){

    setListOfObjects_old();
    //TODO debug info
    std::cout<<"ObjectView complete created!\n";

  }*/

  ObjectView(const ITMLib::ITMRGBDCalib& _calibration, Vector2i _imgSize_rgb, Vector2i _imgSize_d, bool useGPU, ObjCameraPose pose,
                 ObjFloatImage* _depth, ObjUChar4Image* _rgb, LabelImgVector _label_img_vector):
      calibration(_calibration), imgSize_rgb(_imgSize_rgb), imgSize_d(_imgSize_d), camera_Pose(pose),
      depth_Image(_depth), rgb_Image(_rgb), label_img_vector(_label_img_vector){

    setListOfObjects();
    //TODO debug info
    std::cout<<"ObjectView complete created!\n";

  }

  //Destructor
  ~ObjectView(){
//    delete camera_Pose;
    delete rgb_Image;
    delete depth_Image;
//    delete segmentation_Mask;
    if(depth_normal!=NULL){
      delete depth_normal;
    }
/*    for(std::map<int, Object_View_Tuple>::iterator itr = obj_map.begin(); itr != obj_map.end(); itr++){
      delete itr->second;
    }*/
  }


  std::map<int, Object_View_Tuple> getObjMap();

  ObjCameraPose getCameraPose();
  void setCameraPose(ObjCameraPose _pose);




  ObjectView(const ObjectView&);
  ObjectView& operator=(const ObjectView&);
};

}
#endif //MT_OBJSLAM_OBJECTVIEW_NEW_H
