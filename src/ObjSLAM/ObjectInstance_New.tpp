
// Created by khuang on 8/1/18.
//

#include "ObjectInstance_New.h"

namespace ObjSLAM {

template<class TVoxel, class TIndex>
void ObjectInstance_New<TVoxel, TIndex>::addObjectInstanceToLabel() {
  label.get()->addObjectInstance(this->shared_from_this());
}

}