//
// Created by khuang on 6/11/18.
//
#include <iostream>
#include <fstream>
#include <string>

#include "DatasetReader_LPD_Dataset.h"
#include "ObjSLAMDataTypes.h"
#include "ObjectInstanceScene.h"
#include "ObjectView.h"

#include "../../External/InfiniTAM/InfiniTAM/ITMLib/ITMLibDefines.h"

using namespace std;

int main(int argc, char** argv){
  //TODO Debug output
  cout<<"**Hello SLAM World!"<<endl;

  //Path of the depth image file
  string path = argv[1];


  std::istringstream iss( argv[2] );
  //for LPD dataset 1 -13
  int img_number;
  iss>>img_number;
  double time = img_number*0.1;

  //TODO make the path using os path join instead of slash
  string depth_path =path + "/depth/cam0/" + to_string(img_number) + ".exr";
  string rgb_path =path + "/rgb/cam0/" + to_string(img_number) + ".png";
  string label_path =path + "/pixel_label/cam0/" + to_string(img_number) /*+ ".png"*/;
  string pose_path = path + "groundTruthPoseVel_imu.txt";

  //TODO debug
  cout<<"pose_path  "<<pose_path<<endl;

  //create a reader
  DatasetReader_LPD_Dataset reader(640,480);

  reader.setCalib_LPD();
//  cout<<reader.getHeight()<<" "<<reader.getWidth()<<endl;


  ObjSLAM::ObjFloatImage* depth_img = reader.ReadDepth(depth_path);
  ObjSLAM::ObjUChar4Image* rgb_img = reader.ReadRGB(rgb_path);
  ObjSLAM::ObjUIntImage* label_img = reader.ReadLabel(label_path);
  ObjSLAM::LPD_RAW_Pose* pose = reader.ReadPose(pose_path, time);

  //read pose

  //TODO Debug output
  cout <<"** Debug: "<<depth_img->GetElement(0, MEMORYDEVICE_CPU)<<endl;
  cout <<"** Debug: "<<(int)(rgb_img->GetElement(0, MEMORYDEVICE_CPU).r)<<endl;
  cout <<"** Debug: "<<label_img->GetElement(64120, MEMORYDEVICE_CPU)<<endl;
  cout <<"** Debug: "<<pose->qw<<" "<<pose->qx<<endl;



  //create scene:

  //float mu, int maxW, float voxelSize, float viewFrustum_min, float viewFrustum_max, bool stopIntegratingAtMaxW
  ITMLib::ITMSceneParams* params = new ITMLib::ITMSceneParams(0.5,4, 0.01, 0.1,2.0,false );

  //dummy objVector and scene vector:
  //objvector only one obj
//  ObjSLAM::ObjectClassLabel label_table(1, "table");


  //Init View:
  //const ITMLib::ITMRGBDCalib& calibration, Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU, ObjCameraPose pose
  ITMLib::ITMRGBDCalib* calib = reader.getCalib();
  Vector2i imgSize = reader.getSize();
  ObjSLAM::ObjCameraPose dummyPose1;

  ObjSLAM::ObjectView* view0=new ObjSLAM::ObjectView (*calib, imgSize, imgSize, false, dummyPose1, depth_img, rgb_img, label_img);

  //View List
  vector<ObjSLAM::ObjectView*> table_list_view = {view0};

//  ObjSLAM::ObjectInstance* objectInstanceDummy_table = new ObjSLAM::ObjectInstance(label_table);
//  std::vector <ObjSLAM::ObjectInstance> objectVector = {*objectInstanceDummy_table};

  //const ITMLib::ITMSceneParams *_sceneParams, bool _useSwapping, MemoryDeviceType _memoryType, ObjectVector* _objVector, ViewVector* _viewVector
  auto* object = new ObjSLAM::ObjectInstanceScene<ITMVoxel, ITMVoxelIndex>(
      params, true, MEMORYDEVICE_CPU, /*objectVector, */table_list_view);


  return 0;
}
