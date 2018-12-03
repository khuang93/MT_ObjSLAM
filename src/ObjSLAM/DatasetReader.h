//
// Created by khuang on 8/22/18.
//

#ifndef MT_OBJSLAM_DATASETREADER_H
#define MT_OBJSLAM_DATASETREADER_H


#include <fstream>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <memory>
#include "ObjSLAMDataTypes.h"
#include "ObjCameraPose.h"

#include "../../External/InfiniTAM/InfiniTAM/ORUtils/FileUtils.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Camera/ITMRGBDCalib.h"
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Engines/ViewBuilding/Interface/ITMViewBuilder.h>
#include <src/ObjSLAM/ObjSLAMDataTypes.h>

using namespace std;

/**
 * @brief Basic interface to read any kind of data sets.
 */
class DatasetReader {
protected:
    int width, height;
    Vector2i imgSize;
    std::shared_ptr<ITMLib::ITMRGBDCalib> calib;
    string path;
    int img_number = 1;
    ITMLib::ITMViewBuilder *viewBuilder = nullptr;
    std::vector<std::string> LabelFileNames;

public:
    std::shared_ptr<ObjSLAM::ObjUChar4Image> rgb_img;
    std::shared_ptr<ObjSLAM::ObjFloatImage> depth_img;
    std::shared_ptr<ObjSLAM::ObjUIntImage> label_img;
    LabelImgVector label_img_vector;

    /**
     * @brief Constructor
     * @param _path Base path to the dataset
     * @param _imgSize image size as Vector2i
     */
    DatasetReader(string _path, Vector2i _imgSize) : path(_path), imgSize(_imgSize) {
        width = imgSize.x;
        height = imgSize.y;
        string label_path = path + "/pixel_label/";
        GetFileNames(label_path);
    }

    /**
     * @brief Virtual function, reads the next frame and returns the frame number as int.
     * @return frame number
     */
    virtual int ReadNext() = 0;

//  virtual ObjSLAM::ObjShortImage *ConvertToRealDepth(ObjSLAM::ObjFloatImage *depth)=0;

//  virtual bool ReadCalib()=0;

    /**
     * @brief Get all file names in the given directory
     * @param directoryPath
     * @return vector of filenames as strings
     */
    std::vector<std::string> GetFileNames(std::string directoryPath);

    /**
     * @brief Read one Depth image
     * @param Path Path to the depth image
     * @return pointer of ObjFloatImage
     */
    virtual std::shared_ptr<ObjSLAM::ObjFloatImage> ReadOneDepth(std::string Path);

    /**
     * @brief Read one Disparity image
     * @param Path Path to the Disparity image
     * @return pointer of ObjShortImage
     */
    virtual std::shared_ptr<ObjSLAM::ObjShortImage> ReadOneDisparity(std::string Path);

    /**
     * @brief Read one RGB image
     * @param Path Path to the RGB image
     * @return pointer of ObjUChar4Image
     */
    std::shared_ptr<ObjSLAM::ObjUChar4Image> ReadOneRGB(std::string Path);

    /**
     * @brief Read one Label file
     * @param Path Path to the Label file
     * @return pointer of ObjUIntImage
     */
    std::shared_ptr<ObjSLAM::ObjUIntImage> ReadLabel_OneFile(std::string Path);

    /**
     * @brief Read the calibration file in the format of InfiniTAM
     * @param calib_path
     * @return
     */
    virtual bool ReadCalib(string calib_path);


    std::shared_ptr<ITMLib::ITMRGBDCalib> GetCalib();

    void setWidth(int w);

    void setHeight(int h);

    int getWidth();

    int getHeight();

    Vector2i GetSize();

    /**
     * @brief Destructor
     */
    virtual ~DatasetReader() {
        delete viewBuilder;
    }
};

#endif //MT_OBJSLAM_DATASETREADER_H