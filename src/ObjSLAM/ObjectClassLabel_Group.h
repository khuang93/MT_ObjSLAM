//
// Created by khuang on 8/1/18.
//

#ifndef MT_OBJSLAM_OBJECTCLASSLABEL_GROUP_H
#define MT_OBJSLAM_OBJECTCLASSLABEL_GROUP_H

#include <string>
#include <memory>
#include <vector>

namespace ObjSLAM {

    template<typename TVoxel, typename TIndex>
    class ObjectInstance;


/**
 * @brief Class label of objects
 * @tparam TVoxel
 * @tparam TIndex
 */
    template<typename TVoxel, typename TIndex>
    class ObjectClassLabel_Group {
    private:
        //index in the list of all labels
        int LabelIndex;
        //name of the label
        std::string LabelClassName;
        //all object belong to this label are stored in this vector
        std::vector<std::shared_ptr<ObjectInstance<TVoxel, TIndex>>> object_ptr_vector;


    public:

        static const std::vector<std::string> label_list;

        ObjectClassLabel_Group(int _index, std::string _labelClassName) : LabelIndex(_index),
                                                                          LabelClassName(_labelClassName) {}

        ObjectClassLabel_Group(int _index) : LabelIndex(
                _index) { LabelClassName = ObjectClassLabel_Group::label_list.at(LabelIndex); }

        int GetLabelIndex() { return LabelIndex; }

        std::string GetLabelClassName() { return LabelClassName; }

        /**
         * @brief Add a new object to the label
         * @param object_ptr
         */
        void AddObjectInstance(std::shared_ptr<ObjectInstance<TVoxel, TIndex>> object_ptr) {
            this->object_ptr_vector.push_back(object_ptr);
        }

        /**
         * @brief Get the vector of all objects in this label
         * @return
         */
        std::vector<std::shared_ptr<ObjectInstance<TVoxel, TIndex>>> &GetObjPtrVector() { return object_ptr_vector; }


        friend std::ostream &operator<<(std::ostream &stream, ObjectClassLabel_Group<TVoxel, TIndex> &label) {
            return stream << "Class Label of " << label.GetLabelClassName();
        }

    };


}

#include "ObjectClassLabel_Group.tpp"

#endif //MT_OBJSLAM_OBJECTCLASSLABEL_GROUP_H
