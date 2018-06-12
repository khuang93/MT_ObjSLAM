//
// Created by khuang on 6/12/18.
//

#include "DatasetReader_LPD_Dataset.h"


/*static*/
void DatasetReader_LPD_Dataset::SetCalibration_LPD(){

  //cam0
    calib->intrinsics_rgb.SetFrom(640,480, 320, 320, 320, 240);
    calib->intrinsics_d.SetFrom(640,480, 320, 320, 320, 240);
    calib->disparityCalib.SetFrom(0.0, 0.0, ITMLib::ITMDisparityCalib::TRAFO_AFFINE);

}