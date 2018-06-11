//
// Created by khuang on 6/11/18.
//

#ifndef MT_OBJSLAM_DATASETREADER_H
#define MT_OBJSLAM_DATASETREADER_H

//#include "../ObjSLAM/ObjSLAMDataTypes.h"


#include "External/InfiniTAM/InfiniTAM/ITMLib/Utils/ITMImageTypes.h"
#include "External/InfiniTAM/InfiniTAM/ORUtils/Image.h"


//
typedef ORUtils::Image<bool> ObjBoolImage;
//DepthImg
typedef ORUtils::Image<float> ObjFloatImage;
//RGBA Img
typedef ORUtils::Image<Vector4u> ObjUChar4Image;
//Segmentation Mask
typedef ORUtils::Image<uint> ObjUIntImage;


class DatasetReader {


 public:

  static ObjFloatImage Read_TextDepth(std::string PathToFile);
  static ObjUChar4Image Read_RawRGBImage();



};

#endif //MT_OBJSLAM_DATASETREADER_H
