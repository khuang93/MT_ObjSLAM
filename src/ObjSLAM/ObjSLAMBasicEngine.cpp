//
// Created by khuang on 6/29/18.
//

#include "ObjSLAMBasicEngine.h"



namespace ObjSLAM{

using namespace ITMLib;

template <typename TVoxel, typename TIndex>
ITMTrackingState::TrackingResult ObjSLAMBasicEngine<TVoxel,TIndex>::ProcessFrame(ITMUChar4Image *rgbImage, ObjFloatImage *depthImage, ITMIMUMeasurement *imuMeasurement)
{
//  // prepare image and turn it into a depth image
//  if (imuMeasurement == NULL) viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter);
//  else viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, imuMeasurement);

//  if (!mainProcessingActive) return ITMTrackingState::TRACKING_FAILED;

  // tracking
  ORUtils::SE3Pose oldPose(*(trackingState->pose_d));
  if (trackingActive) trackingController->Track(trackingState, view);

  ITMTrackingState::TrackingResult trackerResult = ITMTrackingState::TRACKING_GOOD;
  switch (settings->behaviourOnFailure) {
    case ITMLibSettings::FAILUREMODE_RELOCALISE:
      trackerResult = trackingState->trackerResult;
      break;
    case ITMLibSettings::FAILUREMODE_STOP_INTEGRATION:
      if (trackingState->trackerResult != ITMTrackingState::TRACKING_FAILED)
        trackerResult = trackingState->trackerResult;
      else trackerResult = ITMTrackingState::TRACKING_POOR;
      break;
    default:
      break;
  }

  //relocalisation
  int addKeyframeIdx = -1;
  if (settings->behaviourOnFailure == ITMLibSettings::FAILUREMODE_RELOCALISE)
  {
    if (trackerResult == ITMTrackingState::TRACKING_GOOD && relocalisationCount > 0) relocalisationCount--;

    int NN; float distances;
    view->depth->UpdateHostFromDevice();

    //find and add keyframe, if necessary
    bool hasAddedKeyframe = relocaliser->ProcessFrame(view->depth, trackingState->pose_d, 0, 1, &NN, &distances, trackerResult == ITMTrackingState::TRACKING_GOOD && relocalisationCount == 0);

    //frame not added and tracking failed -> we need to relocalise
    if (!hasAddedKeyframe && trackerResult == ITMTrackingState::TRACKING_FAILED)
    {
      relocalisationCount = 10;

      // Reset previous rgb frame since the rgb image is likely different than the one acquired when setting the keyframe
      view->rgb_prev->Clear();

      const FernRelocLib::PoseDatabase::PoseInScene & keyframe = relocaliser->RetrievePose(NN);
      trackingState->pose_d->SetFrom(&keyframe.pose);

      denseMapper->UpdateVisibleList(view, trackingState, scene, renderState_live, true);
      trackingController->Prepare(trackingState, scene, view, visualisationEngine, renderState_live);
      trackingController->Track(trackingState, view);

      trackerResult = trackingState->trackerResult;
    }
  }

  bool didFusion = false;
  if ((trackerResult == ITMTrackingState::TRACKING_GOOD || !trackingInitialised) && (fusionActive) && (relocalisationCount == 0)) {
    // fusion
    denseMapper->ProcessFrame(view, trackingState, scene, renderState_live);
    didFusion = true;
    if (framesProcessed > 50) trackingInitialised = true;

    framesProcessed++;
  }

  if (trackerResult == ITMTrackingState::TRACKING_GOOD || trackerResult == ITMTrackingState::TRACKING_POOR)
  {
    if (!didFusion) denseMapper->UpdateVisibleList(view, trackingState, scene, renderState_live);

    // raycast to renderState_live for tracking and free visualisation
    trackingController->Prepare(trackingState, scene, view, visualisationEngine, renderState_live);

    if (addKeyframeIdx >= 0)
    {
      ORUtils::MemoryBlock<Vector4u>::MemoryCopyDirection memoryCopyDirection =
          settings->deviceType == ITMLibSettings::DEVICE_CUDA ? ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CUDA : ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU;

      kfRaycast->SetFrom(renderState_live->raycastImage, memoryCopyDirection);
    }
  }
  else *trackingState->pose_d = oldPose;

#ifdef OUTPUT_TRAJECTORY_QUATERNIONS
  const ORUtils::SE3Pose *p = trackingState->pose_d;
	double t[3];
	double R[9];
	double q[4];
	for (int i = 0; i < 3; ++i) t[i] = p->GetInvM().m[3 * 4 + i];
	for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c)
		R[r * 3 + c] = p->GetM().m[c * 4 + r];
	QuaternionFromRotationMatrix(R, q);
	fprintf(stderr, "%f %f %f %f %f %f %f\n", t[0], t[1], t[2], q[1], q[2], q[3], q[0]);
#endif

  return trackerResult;
}

}