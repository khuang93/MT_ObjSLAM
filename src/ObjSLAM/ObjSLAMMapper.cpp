//
// Created by khuang on 6/11/18.
//
#include <iostream>
#include <fstream>
#include <tuple>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderStateFactory.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Core/ITMBasicEngine.h>

#include "DatasetReader_LPD_Dataset.h"
#include "ObjectInstanceScene.h"
#include "ObjectView_old.h"
#include "ObjSLAMBasicEngine.h"

#include "../../External/InfiniTAM/InfiniTAM/ITMLib/ITMLibDefines.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Tracking/ITMTrackingState.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderState.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderState_VH.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Engines/Reconstruction/CPU/ITMSceneReconstructionEngine_CPU.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Engines/Reconstruction/CUDA/ITMSceneReconstructionEngine_CUDA.h"
//#include "External/InfiniTAM/InfiniTAM/ITMLib/Engines/Reconstruction/Interface/ITMSceneReconstructionEngine.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Engines/Reconstruction/ITMSceneReconstructionEngineFactory.h"

#include "External/InfiniTAM/InfiniTAM/ITMLib/Engines/Visualisation/CPU/ITMVisualisationEngine_CPU.h"

using namespace std;

int main(int argc, char **argv) {
  //TODO Debug output
  cout << "**Hello SLAM World!" << endl;

  //Path of the depth image file
  string path = argv[1];

  std::istringstream iss(argv[2]);
  //for LPD dataset 1 -13
  int img_number;
  iss >> img_number;
  double time = img_number * 0.1;

  //TODO make the path using os path join instead of slash
  string depth_path = path + "/depth/cam0/" + to_string(img_number) + ".exr";
  string rgb_path = path + "/rgb/cam0/" + to_string(img_number) + ".png";
  string normal_path = path + "/normal/cam0/" + to_string(img_number) + ".png";
  string label_path = path + "/pixel_label/cam0/" + to_string(img_number) + ".txt";
  string pose_path = path + "groundTruthPoseVel_imu.txt";

  //TODO debug
  cout << "pose_path  " << pose_path << endl;

  //create a reader and read inputs
  DatasetReader_LPD_Dataset reader(640, 480);

  reader.setCalib_LPD();

//  depth
  ObjSLAM::ObjFloatImage *ray_depth_img = reader.ReadDepth(depth_path);
  ObjSLAM::ObjFloatImage *depth_img = reader.convertRayDepthToZDepth(ray_depth_img);
  const char *name = (to_string(img_number) + ".pgm").c_str();
//  SaveImageToFile(depth_img, name);

// RGB
  ObjSLAM::ObjUChar4Image *rgb_img = reader.ReadRGB(rgb_path);
  const char *name_rgb = (to_string(img_number) + ".ppm").c_str();
//  SaveImageToFile(rgb_img, name_rgb);
//  ObjSLAM::ObjFloat4Image* depth_normal = reader.ReadNormal(normal_path);
//  Label
  ObjSLAM::ObjUIntImage *label_img = reader.ReadLabel_OneFile(label_path);
  ObjSLAM::LPD_RAW_Pose *raw_pose = reader.ReadLPDRawPose(pose_path, time);
  //T_bw
  ObjSLAM::ObjCameraPose *T_bw = reader.convertRawPose_to_Pose(raw_pose);
  //T_cb
  ObjSLAM::ObjCameraPose *T_cb = new ObjSLAM::ObjCameraPose(0.5, -0.5, 0.5, -0.5, 0, 0, 0);
  auto * T_cw_SE3 = new ORUtils::SE3Pose(T_cb->getSE3Pose()->GetM()*T_bw->getSE3Pose()->GetM());
  auto *pose = new ObjSLAM::ObjCameraPose(T_cw_SE3);

//  auto* image_s = new ObjSLAM::ObjShortImage();
//    reader.calculateDisparityFromDepth(depth_img);
//  SaveImageToFile(image_s, "short_depth");


  //TODO Debug output
  cout << "** Debug: " << depth_img->GetElement(10, MEMORYDEVICE_CPU) << endl;
  cout << "** Debug: " << (int) (rgb_img->GetElement(0, MEMORYDEVICE_CPU).r) << endl;

  cout << "** Debug: " << label_img->GetElement(64120, MEMORYDEVICE_CPU) << endl;
  cout << "** Debug: " << raw_pose->qw << " " << raw_pose->qx << endl;
  cout << "** Debug: " << pose->getSE3Pose()->GetT().x << endl;



  //create scene:

  //float mu, int maxW, float voxelSize, float viewFrustum_min, float viewFrustum_max, bool stopIntegratingAtMaxW
  ITMLib::ITMSceneParams *params = new ITMLib::ITMSceneParams(0.5, 4, 0.01, 0.1, 2.0, false);


  //Init View:
  //const ITMLib::ITMRGBDCalib& calibration, Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU, ObjCameraPose pose
  ITMLib::ITMRGBDCalib *calib = reader.getCalib();
  Vector2i imgSize = reader.getSize();

  auto *view0 = new ObjSLAM::ObjectView_New(*calib, imgSize, imgSize, false, *pose, depth_img, rgb_img, label_img);

  //View List
  vector<ObjSLAM::ObjectView_New *> table_list_view = {view0};

  //const ITMLib::ITMSceneParams *_sceneParams, bool _useSwapping, MemoryDeviceType _memoryType, ObjectVector* _objVector, ViewVector* _viewVector
  auto *object = new ObjSLAM::ObjectInstanceScene<ITMVoxel, ITMVoxelIndex>(
      params, true, MEMORYDEVICE_CPU, /*objectVector, */table_list_view);


  //Tracking State
  auto *trackingState = new ITMLib::ITMTrackingState(imgSize, MEMORYDEVICE_CPU);
  trackingState->pose_d = pose->getSE3Pose();

  //RenderState
  auto *renderState = new ITMLib::ITMRenderState_VH(1,
                                                    imgSize,
                                                    object->sceneParams->viewFrustum_min,
                                                    object->sceneParams->viewFrustum_min,
                                                    MEMORYDEVICE_CPU);



  auto *engine_cpu = new ITMLib::ITMSceneReconstructionEngine_CPU<ITMVoxel, ITMVoxelIndex>;
//  auto* engine_gpu = new ITMLib::ITMSceneReconstructionEngine_CUDA<ITMVoxel, ITMVoxelIndex>;

//  This gives weird linker errors
//  ITMLib::ITMSceneReconstructionEngine<ITMVoxel, ITMVoxelIndex>* engine2 = ITMLib::ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<ITMVoxel,ITMVoxelIndex>(ITMLib::ITMLibSettings::DEVICE_CPU);
  engine_cpu->ResetScene(object);

  ObjSLAM::Object_View_Tuple view_tuple = view0->getObjMap().find(58)->second;

  engine_cpu->AllocateSceneFromDepth((ITMLib::ITMScene<ITMVoxel, ITMVoxelIndex> *) object,
                                     std::get<1>(view_tuple),
                                     trackingState,
                                     renderState);
  engine_cpu->IntegrateIntoScene(object, std::get<1>(view_tuple), trackingState, renderState);

  cout << std::get<0>(view_tuple)->getClassLabel().getLabelIndex() << endl;  std::cout << "DEBUG" << std::endl;
  cout << std::get<1>(view_tuple)->depth->GetElement(154610, MEMORYDEVICE_CPU) << endl;
  cout << (int) (std::get<1>(view_tuple)->rgb->GetElement(154610, MEMORYDEVICE_CPU).w) << endl;

  cout << "Scene Integration finish\n";

  SaveImageToFile(std::get<1>(view_tuple)->depth, "DEPTH");
  SaveImageToFile(std::get<1>(view_tuple)->rgb, "RGB");

//visualize
  ObjSLAM::ObjUChar4Image *img;
  auto *vis_eng_cpu = new ITMLib::ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>;







//
//  vis_eng_cpu->FindVisibleBlocks(scene, pose, intrinsics, renderState_freeview);
//  vis_eng_cpu->CreateExpectedDepths(scene, pose, intrinsics, renderState_freeview);
//  vis_eng_cpu->RenderImage(scene, pose, intrinsics, renderState_freeview, renderState_freeview->raycastImage, type);

//
  vis_eng_cpu->FindVisibleBlocks((ITMLib::ITMScene<ITMVoxel, ITMVoxelIndex>*)object, pose->getSE3Pose(), &(calib->intrinsics_d), renderState);
  vis_eng_cpu->CreateExpectedDepths((ITMLib::ITMScene<ITMVoxel, ITMVoxelIndex>*)object, pose->getSE3Pose(), &(calib->intrinsics_d), renderState);
  vis_eng_cpu->RenderImage((ITMLib::ITMScene<ITMVoxel, ITMVoxelIndex>*)object, pose->getSE3Pose(), &(calib->intrinsics_d),renderState,img);

  SaveImageToFile(img,"recon.ppm");

  ITMLib::ITMLibSettings *internalSettings = new ITMLib::ITMLibSettings();

  auto* basicEngine = ObjSLAM::ObjSLAMBasicEngine<ITMVoxel,ITMVoxelIndex>(internalSettings, *calib, imgSize, imgSize);

//  delete params;
//  delete depth_img;
//  delete rgb_img;
//  delete raw_pose;
//  delete pose;
//  delete view0;
//  delete view0_new;

  return 0;
}
