//
// Created by khuang on 9/24/18.
//

#include "ObjSLAMMainEngine.h"


//static global variables
//bool saveSTL = false;
//int STL_Frequency = 1;
//int reader_SkipFrames = 0;
//int numthreads = 4;
//int totFrames;


int ObjSLAMMainEngine::ReadNext() {
    imgNum = reader->ReadNext();
    if (imgNum == -1) return -1; //no more image
    sceneIsBackground = true;

    wholeView->depth->SetFrom(reader->depth_img, ORUtils::Image<float>::CPU_TO_CPU);
    wholeView->rgb->SetFrom(reader->rgb_img, ORUtils::Image<Vector4u>::CPU_TO_CPU);
    bool isKeyFrame = true;
//   if(isKeyFrame){
    label_img_vector = reader->label_img_vector;
    depth_img = reader->depth_img;
    rgb_img = reader->rgb_img;
//   }
    framesElapsedBeforeMapping++;
    return imgNum;
}

void ObjSLAMMainEngine::TrackFrame() {
    t_state = trackingEngine->TrackFrame(wholeView.get());
}


void ObjSLAMMainEngine::MapFrame() {
    mappingEngine->ProcessFrame();
    mappingEngine->RenderAllObjImages();
    mapperFree = true;
}

void ObjSLAMMainEngine::OutputPics() {
    mappingEngine->OutputAllObjImages();

}

void ObjSLAMMainEngine::UpdateMappingEngine() {

    mapperFree = false;
    mappingEngine->UpdateImgNumber(imgNum);
    framesElapsedBeforeMapping = 0;
    mappingEngine->UpdateTrackingState(t_state);
    mappingEngine->CreateView(depth_img, rgb_img, label_img_vector);
}

ObjSLAM::ObjUChar4Image *ObjSLAMMainEngine::GetImage(int n) {
    if (n < this->mappingEngine->number_activeObjects) {
        return mappingEngine->GetImage(n);
    } else {
        return mappingEngine->GetImage(0);
    }
}

ObjSLAM::ObjUChar4Image *ObjSLAMMainEngine::GetInputImage() {
    return this->wholeView->rgb;
}

ObjSLAM::ObjUChar4Image *ObjSLAMMainEngine::GetBGImage() {
    return this->mappingEngine->GetBGImage();
}


ObjSLAM::ObjUChar4Image *ObjSLAMMainEngine::GetAboveImage() {
    return this->mappingEngine->GetImageFromAbove();
}