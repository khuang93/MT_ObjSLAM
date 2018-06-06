//
// Created by khuang on 6/6/18.
//

#ifndef MT_OBJSLAM_OBJECTCLASSLABEL_H
#define MT_OBJSLAM_OBJECTCLASSLABEL_H
#include <string>
#include <iostream>
namespace ObjSLAM{

class ObjectClassLabel {
 private:
  int LabelIndex;
  std::string LabelClassName;

 public:
  ObjectClassLabel(int _index, std::string _labelClassName):LabelIndex(_index), LabelClassName(_labelClassName){
    std::cout << "ObjectClassLabel "<< LabelClassName << " created! \n";
  }
  
  int getLabelIndex();
  
  std::string getLabelClassName();
  
};

}


#endif //MT_OBJSLAM_OBJECTCLASSLABEL_H
