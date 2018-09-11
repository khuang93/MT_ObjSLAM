//
// Created by khuang on 8/1/18.
//

#include "ObjectClassLabel_Group.h"
//#include "ObjectInstance_New.tpp"

namespace ObjSLAM{
template<typename TVoxel, typename TIndex>
const std::vector<std::string> ObjectClassLabel_Group<TVoxel,TIndex>::label_list= {"BG", "person", "bicycle", "car", "motorcycle", "airplane",
"bus", "train", "truck", "boat", "traffic light",
"fire hydrant", "stop sign", "parking meter", "bench", "bird",
"cat", "dog", "horse", "sheep", "cow", "elephant", "bear",
"zebra", "giraffe", "backpack", "umbrella", "handbag", "tie",
"suitcase", "frisbee", "skis", "snowboard", "sports ball",
"kite", "baseball bat", "baseball glove", "skateboard",
"surfboard", "tennis racket", "bottle", "wine glass", "cup",
"fork", "knife", "spoon", "bowl", "banana", "apple",
"sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza",
"donut", "cake", "chair", "couch", "potted plant", "bed",
"dining table", "toilet", "tv", "laptop", "mouse", "remote",
"keyboard", "cell phone", "microwave", "oven", "toaster",
"sink", "refrigerator", "book", "clock", "vase", "scissors",
"teddy bear", "hair drier", "toothbrush"};

/*template<class TVoxel, class TIndex>
void ObjectClassLabel_Group<TVoxel,TIndex>::addObjectInstance(ObjectInstance_New<TVoxel, TIndex>* object_ptr){

  std::shared_ptr<ObjectInstance_New<TVoxel, TIndex>> ptr(object_ptr);
  this->object_ptr_vector.push_back(ptr);
}*/

}