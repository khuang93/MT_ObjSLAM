//
// Created by khuang on 6/9/18.
//
#pragma once
#ifndef MT_OBJSLAM_OBJSLAMDATATYPES_H
#define MT_OBJSLAM_OBJSLAMDATATYPES_H

#include "External/InfiniTAM/InfiniTAM/ITMLib/Utils/ITMImageTypes.h"
#include "External/InfiniTAM/InfiniTAM/ORUtils/Image.h"
#include "ObjectInstance.h"

namespace ObjSLAM {

//
typedef ORUtils::Image<bool> ObjBoolImage;
//DepthImg
typedef ORUtils::Image<float> ObjFloatImage;
//Disparity
typedef ORUtils::Image<short> ObjShortImage;
//RGBA Img
typedef ORUtils::Image<Vector4u> ObjUChar4Image;
typedef ORUtils::Image<Vector4f> ObjFloat4Image;

//Segmentation Mask
typedef ORUtils::Image<uint> ObjUIntImage;

typedef ORUtils::Matrix4<float> ObjMatrix4f;

template<class TVoxel, class TIndex>
using ObjectInstance_ptr=std::shared_ptr<ObjectInstance<TVoxel, TIndex>>;



template<typename TVoxel, typename TIndex>
using Object_View_Tup = std::tuple<std::shared_ptr<ObjectInstance<TVoxel, TIndex>>,
                                   std::shared_ptr<ITMLib::ITMView>>;

//
//typedef ORUtils::Image<short> ObjShortImage;
//typedef ORUtils::Image<Vector2s> ObjShort2Image;
//typedef ORUtils::Image<Vector3s> ObjShort3Image;
//typedef ORUtils::Image<Vector4s> ObjShort4Image;
//typedef ORUtils::Image<uchar> ObjUCharImage;
//
//
//typedef ORUtils::Image<ushort> ObjUShortImage;


}
typedef std::vector<std::shared_ptr<ObjSLAM::ObjUIntImage>> LabelImgVector;

#endif //MT_OBJSLAM_OBJSLAMDATATYPES_H
