//
// Created by khuang on 6/11/18.
//
#include <iostream>
#include <fstream>
#include <string>

#include "DatasetReader_LPD_Dataset.h"
#include "ObjSLAMDataTypes.h"
#include "ObjectInstanceScene.h"
#include "ObjectView_old.h"

#include "../../External/InfiniTAM/InfiniTAM/ITMLib/ITMLibDefines.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Tracking/ITMTrackingState.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderState.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderState_VH.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Engines/Reconstruction/CPU/ITMSceneReconstructionEngine_CPU.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Engines/Reconstruction/CUDA/ITMSceneReconstructionEngine_CUDA.h"
//#include "External/InfiniTAM/InfiniTAM/ITMLib/Engines/Reconstruction/Interface/ITMSceneReconstructionEngine.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Scene/ITMVoxelBlockHash.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Scene/ITMPlainVoxelArray.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Engines/Reconstruction/ITMSceneReconstructionEngineFactory.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Utils/ITMLibSettings.h"
#include "ObjectView_New.h"

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
  ObjSLAM::LPD_RAW_Pose* raw_pose = reader.ReadLPDRawPose(pose_path, time);
  ObjSLAM::ObjCameraPose* pose = reader.convertRawPose_to_Pose(raw_pose);
  //read pose

  //TODO Debug output
  cout <<"** Debug: "<<depth_img->GetElement(0, MEMORYDEVICE_CPU)<<endl;
  cout <<"** Debug: "<<(int)(rgb_img->GetElement(0, MEMORYDEVICE_CPU).r)<<endl;
  cout <<"** Debug: "<<label_img->GetElement(64120, MEMORYDEVICE_CPU)<<endl;
  cout <<"** Debug: "<<raw_pose->qw<<" "<<raw_pose->qx<<endl;
  cout <<"** Debug: "<<pose->getSE3Pose()->GetT().x<<endl;



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
//  ObjSLAM::ObjCameraPose dummyPose1;

  ObjSLAM::ObjectView_old* view0=new ObjSLAM::ObjectView_old (*calib, imgSize, imgSize, false, *pose, depth_img, rgb_img, label_img);

  ObjSLAM::ObjectView_New* view0_new=new ObjSLAM::ObjectView_New (*calib, imgSize, imgSize, false, *pose, depth_img, rgb_img, label_img);

  //View List
  vector<ObjSLAM::ObjectView_old*> table_list_view = {view0};

//  ObjSLAM::ObjectInstance* objectInstanceDummy_table = new ObjSLAM::ObjectInstance(label_table);
//  std::vector <ObjSLAM::ObjectInstance> objectVector = {*objectInstanceDummy_table};

  //const ITMLib::ITMSceneParams *_sceneParams, bool _useSwapping, MemoryDeviceType _memoryType, ObjectVector* _objVector, ViewVector* _viewVector
  auto* object = new ObjSLAM::ObjectInstanceScene<ITMVoxel, ITMVoxelIndex>(
      params, true, MEMORYDEVICE_CPU, /*objectVector, */table_list_view);


  //Tracking State
  auto* trackingState = new ITMLib::ITMTrackingState(imgSize, MEMORYDEVICE_CPU);
//  trackingState->pose_d = pose->getSE3Pose();

  //RenderState
  auto* renderState = new ITMLib::ITMRenderState_VH(1,imgSize, object->sceneParams->viewFrustum_min, object->sceneParams->viewFrustum_min, MEMORYDEVICE_CPU);

  auto* engine_cpu = new ITMLib::ITMSceneReconstructionEngine_CPU<ITMVoxel, ITMVoxelIndex>;
//  auto* engine_gpu = new ITMLib::ITMSceneReconstructionEngine_CUDA<ITMVoxel, ITMVoxelIndex>;

//  This gives weird linker errors
//  ITMLib::ITMSceneReconstructionEngine<ITMVoxel, ITMVoxelIndex>* engine2 = ITMLib::ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<ITMVoxel,ITMVoxelIndex>(ITMLib::ITMLibSettings::DEVICE_CPU);
  engine_cpu->ResetScene(object);

  engine_cpu->AllocateSceneFromDepth((ITMLib::ITMScene<ITMVoxel, ITMVoxelIndex>*)object,(ITMLib::ITMView*)view0,trackingState,renderState);
  engine_cpu->IntegrateIntoScene(object,view0,trackingState,renderState);

//  cout<<view0_new->getObjMap().find(10)->second.first->getClassLabel().getLabelIndex()<<endl;

  cout<<"Scene Integration finish\n";

  return 0;
}
