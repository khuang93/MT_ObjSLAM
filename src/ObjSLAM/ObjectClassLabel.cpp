//
// Created by khuang on 6/6/18.
//

#include "ObjectClassLabel.h"

namespace ObjSLAM{


  int ObjectClassLabel::getLabelIndex(){
    return LabelIndex;
  }

  std::string ObjectClassLabel::getLabelClassName(){
    return LabelClassName;
  }


}