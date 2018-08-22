//
// Created by khuang on 8/22/18.
//
#include "DatasetReader.h"

std::vector<std::string> DatasetReader::getFileNames(std::string directoryPath) {
  namespace fs = boost::filesystem;
  std::vector<std::string> names;

  if (fs::exists(directoryPath)) {
    fs::directory_iterator it(directoryPath);
    fs::directory_iterator end;

    while (it != end) {
      names.push_back(it->path().filename().string());
      ++it;
    }
  }

  return names;
}


ObjSLAM::ObjFloatImage *DatasetReader::ReadOneDepth(std::string Path) {
  ifstream in;

  in.open(Path);

  vector<float> vector_in;

  while (in.peek() != EOF) {
    float tmp;
    in >> tmp;
    vector_in.push_back(tmp);
  }

  //TODO debug output
//    cout << "**Input size is " << vector_in.size() << endl;

//    ORUtils::Vector2<int> imgSize(width, height);
  auto *res = new ObjSLAM::ObjFloatImage(imgSize, MEMORYDEVICE_CPU);

  res->ChangeDims(imgSize);


//
  for (int i = 0; i < height; i++) {
//      cout<<"i"<<i<<endl;
    for (int j = 0; j < width; j++) {
//        res[height * i + j] = vector_in.at(height * i + j);

      res->GetData(MEMORYDEVICE_CPU)[width * i + j] = vector_in.at(width * i + j); //in meters
    }

  }

//    SaveImageToFile(res, "testD");
  return res;
}

ObjSLAM::ObjShortImage *DatasetReader::ReadOneDisparity(std::string Path) {
  ifstream in;

  in.open(Path);
  //read rgb from png file



  auto *res = new ObjSLAM::ObjShortImage(imgSize, MEMORYDEVICE_CPU);

  res->ChangeDims(imgSize);

  ReadImageFromFile(res, Path.c_str());

//    SaveImageToFile(res, "testRGB");

  return res;
}

ObjSLAM::ObjUChar4Image *DatasetReader::ReadOneRGB(std::string Path){
  ifstream in;

  in.open(Path);
  //read rgb from png file



  auto *res = new ObjSLAM::ObjUChar4Image(imgSize, MEMORYDEVICE_CPU);

  res->ChangeDims(imgSize);

  ReadImageFromFile(res, Path.c_str());

//    SaveImageToFile(res, "testRGB");

  return res;
}


std::shared_ptr<ObjSLAM::ObjUIntImage> DatasetReader::ReadLabel_OneFile(std::string Path) {
  ifstream in;

  in.open(Path);

  vector<unsigned int> vector_in;

  while (in.peek() != EOF) {
    float tmp;
    in >> tmp;
    vector_in.push_back(tmp);
  }

  //TODO debug output
//    cout << "**Input size of label is " << vector_in.size() << endl;

  //set dimension
//    ORUtils::Vector2<int> imgSize(width, height);

  //create UintImage object
//    auto *res = new ObjSLAM::ObjUIntImage(imgSize, MEMORYDEVICE_CPU);
  std::shared_ptr<ObjSLAM::ObjUIntImage> res = std::make_shared<ObjSLAM::ObjUIntImage>(imgSize,MEMORYDEVICE_CPU);

  res->ChangeDims(imgSize);


//
  for (int i = 0; i < width; i++) {
//      cout<<"i"<<i<<endl;
    for (int j = 0; j < height; j++) {
//        res[height * i + j] = vector_in.at(height * i + j);

      res->GetData(MEMORYDEVICE_CPU)[height * i + j] = vector_in.at(height * i + j);
    }

  }
  return res;
}

ITMLib::ITMRGBDCalib *DatasetReader::getCalib() {
  return calib;
}

void DatasetReader::setWidth(int w) {
  width = w;
}
void DatasetReader::setHeight(int h) {
  height = h;
}
int DatasetReader::getWidth() {
  return width;
}
int DatasetReader::getHeight() {
  return height;
}
Vector2i DatasetReader::getSize() {
  return imgSize;
}