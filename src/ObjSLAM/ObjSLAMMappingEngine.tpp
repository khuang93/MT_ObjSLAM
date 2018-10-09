//
// Created by khuang on 7/5/18.
//

#pragma once

#include "ObjSLAMMappingEngine.h"

#include <math.h>

#include "ObjSLAMVoxelSceneParams.h"
#include <omp.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/Scene/ITMVoxelBlockHash.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderState_VH.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/Scene/ITMRepresentationAccess.h>

//define the global variable


namespace ObjSLAM {

    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::CreateView(ObjFloatImage *_depth,
                                                          ObjUChar4Image *_rgb,
                                                          LabelImgVector _label_img_vector) {

        if (settings->deviceType != ITMLib::ITMLibSettings::DEVICE_CUDA) {
            this->view = std::make_shared<ObjectView<TVoxel, TIndex>>(*calib,
                                                                      imgSize,
                                                                      imgSize,
                                                                      false,
                                                                      _depth,
                                                                      _rgb,
                                                                      _label_img_vector);
            this->view_vec.push_back(view);

        } else {
            this->view = std::make_shared<ObjectView<TVoxel, TIndex>>(*calib,
                                                                      imgSize,
                                                                      imgSize,
                                                                      false,
                                                                      _depth,
                                                                      _rgb,
                                                                      _label_img_vector);
            this->view_vec.push_back(view);
        }

        this->UpdateViewPose();
//  this->UpdateObjBoolImg();


        //init all objects in view
        view->SetListOfObjects(label_ptr_vector);

    }

    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::ProcessFrame() {

        bool useSwapping = (settings->swappingMode != ITMLib::ITMLibSettings::SWAPPINGMODE_DISABLED);

        cout << "ProcessFrame...\n";

        if (view->GetObjVec().size() > 0) {

            bool newObject = true;

            int number_free_preallocation = false; // this->number_preallocation - number_totalObjects;

            bool usePreallocation = number_free_preallocation > view->GetObjVec().size();
#ifdef WITH_OPENMP
#pragma omp parallel for private(sceneIsBackground) /*num_threads(numthreads)*/
#endif
            for (int t = 0; t < view->GetObjVec().size(); t++) {

                Object_View_Tup<TVoxel, TIndex> view_tuple = view->GetObjVec().at(t);

                auto obj_inst_ptr = std::get<0>(view_tuple);
                auto label_ptr = obj_inst_ptr->GetClassLabel();

                int labelIndex = label_ptr->GetLabelIndex();

                if (labelIndex == 74) continue; //skip books

                //set the flag of background or not
                sceneIsBackground = labelIndex == 0 ? true : false;
//                cout<<"sceneIsBackground "<<sceneIsBackground<<endl;

                std::shared_ptr<ITMLib::ITMView> itmview = std::get<1>(view_tuple);

                auto &obj_ptr_vec_val = label_ptr->GetObjPtrVector();
                auto obj_ptr_vec = &obj_ptr_vec_val;


                if (obj_ptr_vec->size() == 0) {
                    obj_inst_ptr->AddObjectInstanceToLabel();

                    newObject = true;
                } else {

                    for (size_t i = 0; i < obj_ptr_vec->size(); ++i) {
                        ObjectInstance_ptr<TVoxel, TIndex> existing_obj_ptr = obj_ptr_vec->at(i);
                        if (!existing_obj_ptr->isVisible) continue;

                        if (obj_inst_ptr->CheckIsBackground()) {
                            newObject = false;
                        } else if (existing_obj_ptr->isVisible) {
                            sceneIsBackground == false;
                            newObject = !this->CheckIsSameObject2D(existing_obj_ptr, obj_inst_ptr);
//                            cout<<"sceneIsBackground "<<sceneIsBackground<<endl;
                        }
                        if (!newObject) {
                            //this is an existing object, no need to compare with further objs
//            obj_inst_scene_ptr = existing_obj_ptr->GetScene();

                            obj_inst_ptr = existing_obj_ptr;

                            break;
                        }
                    }
                    if (newObject) {
                        obj_inst_ptr->AddObjectInstanceToLabel();
                        obj_inst_ptr_vector.push_back(obj_inst_ptr);
                    }
                }


                obj_inst_ptr->SetCurrentView(itmview);

                if (newObject) {
#pragma omp critical
                    {
                        if (obj_inst_ptr->CheckIsBackground()) {
                            obj_inst_ptr_vector.insert(obj_inst_ptr_vector.begin(), obj_inst_ptr);
                            active_obj_ptr_vector.insert(active_obj_ptr_vector.begin(), obj_inst_ptr);
                        } else {
                            obj_inst_ptr_vector.push_back(obj_inst_ptr);
                            active_obj_ptr_vector.push_back(obj_inst_ptr);
                        }
                        number_activeObjects = active_obj_ptr_vector.size();
                        number_totalObjects = obj_inst_ptr_vector.size();
                    }
                    sceneIsBackground = obj_inst_ptr->CheckIsBackground();
//                    obj_inst_ptr->view_count = 1;
                    if (obj_inst_ptr->CheckIsBackground() && BG_object_ptr.get() == nullptr) {
                        BG_object_ptr = obj_inst_ptr;
                    }


                    if (!usePreallocation) {
                        auto obj_inst_scene_ptr = std::make_shared<ObjectInstanceScene<TVoxel, TIndex>>(
                                sceneParams_ptr.get(),
                                useSwapping,
                                MEMORYDEVICE_CPU);
                        obj_inst_ptr->SetScene(obj_inst_scene_ptr);
                        denseMapper->ResetScene(obj_inst_ptr->GetScene().get());
                    } else {
//                        obj_inst_ptr->SetScene(preAllocation_array[number_totalObjects]);
                    }


                    std::shared_ptr<ITMLib::ITMRenderState> renderState_ptr(
                            new ITMLib::ITMRenderState_VH((sceneIsBackground
                                                           ? ITMLib::ITMVoxelBlockHash::noTotalEntries_BG
                                                           : ITMLib::ITMVoxelBlockHash::noTotalEntries),
                                                          imgSize,
                                                          settings->sceneParams.viewFrustum_min,
                                                          settings->sceneParams.viewFrustum_max,
                                                          MEMORYDEVICE_CPU));
                    std::shared_ptr<ITMLib::ITMRenderState> renderState_above_ptr(
                            new ITMLib::ITMRenderState_VH((sceneIsBackground
                                                           ? ITMLib::ITMVoxelBlockHash::noTotalEntries_BG
                                                           : ITMLib::ITMVoxelBlockHash::noTotalEntries),
                                                          imgSize,
                                                          settings->sceneParams.viewFrustum_min,
                                                          settings->sceneParams.viewFrustum_max,
                                                          MEMORYDEVICE_CPU));
                    obj_inst_ptr->SetRenderState(renderState_ptr, renderState_above_ptr);

                    auto t_state_ptr = std::make_shared<ITMLib::ITMTrackingState>(imgSize, MEMORYDEVICE_CPU);
                    obj_inst_ptr->SetTrackingState(t_state_ptr);

//                    denseMapper->ResetScene(obj_inst_scene_ptr.get());

                } else {
//                    obj_inst_ptr->view_count++;
//                    cout << "obj_view_count = " << obj_inst_ptr->view_count << endl;
                }


                //ProcessOneObject
                ProcessOneObject(itmview, obj_inst_ptr);

                //re-init the bool of newObject
                newObject = true;
            }
        }
        //Insert function: prepare tracking with all objs
        this->renderState_RenderAll->raycastResult->Clear();
        t_controller->Prepare(t_state.get(), this->renderState_RenderAll.get(), this->obj_inst_ptr_vector,
                              visualisationEngine); //visualisationEngine_BG

        write2PLYfile(this->renderState_RenderAll->raycastResult, "raycast_img" + to_string(imgNumber) + ".ply");
        write2PLYfile(BG_object_ptr->GetTrackingState()->pointCloud->locations,
                      "t_state_PCL_img" + to_string(imgNumber) + ".ply");
//  PrepareTrackingWithAllObj();
    }


    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::ProcessOneObject(std::shared_ptr<ITMLib::ITMView> &itmview,
                                                                ObjectInstance_ptr <TVoxel, TIndex> obj_inst_ptr) {


        auto scene = obj_inst_ptr->GetScene().get();
        obj_inst_ptr->GetRenderState()->raycastResult->Clear();
        obj_inst_ptr->GetRenderState()->raycastImage->Clear();

        if (!obj_inst_ptr.get()->CheckIsBackground()) {
            sceneIsBackground = false;
//            cout<<"sceneIsBackground "<<sceneIsBackground<<endl;
            std::shared_ptr<ITMLib::ITMTrackingState> tmp_t_state = obj_inst_ptr->GetTrackingState();

            tmp_t_state->Reset();

            tmp_t_state->pose_d->SetFrom(this->t_state->pose_d);
            tmp_t_state->trackerResult = ITMLib::ITMTrackingState::TRACKING_GOOD;

            denseMapper->ProcessFrame(itmview.get(), tmp_t_state.get(), scene, obj_inst_ptr->GetRenderState().get(),
                                      true);

            denseMapper->UpdateVisibleList(itmview.get(), tmp_t_state.get(), scene,
                                           obj_inst_ptr->GetRenderState().get(), true);
            obj_inst_ptr->GetRenderState()->raycastResult->Clear();
            t_controller->Prepare(tmp_t_state.get(),
                                  scene,
                                  obj_inst_ptr.get()->GetAnchorView_ITM(),
                                  visualisationEngine,
                                  obj_inst_ptr->GetRenderState().get());

        } else {
            sceneIsBackground = true;
//            cout<<"sceneIsBackground "<<sceneIsBackground<<endl;
            this->t_state->trackerResult = ITMLib::ITMTrackingState::TRACKING_GOOD;
            denseMapper->ProcessFrame(BG_object_ptr->GetCurrentView().get(),
                                      this->t_state.get(),
                                      scene,
                                      this->renderState_RenderAll.get(),
                                      true);

            denseMapper->UpdateVisibleList(BG_object_ptr->GetCurrentView().get(),
                                           this->t_state.get(),
                                           scene,
                                           this->renderState_RenderAll.get(),
                                           true);
            BG_object_ptr->GetRenderState()->raycastResult->Clear();
            //BG renderstate
            t_controller->Prepare(this->t_state.get(),
                                  scene,
                                  BG_object_ptr->GetAnchorView_ITM(),
                                  visualisationEngine,
                                  BG_object_ptr->GetRenderState().get()); //visualisationEngine_BG

        }

    }

    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::UpdateVisibilityOfObj(ObjectInstance_ptr <TVoxel, TIndex> obj_inst_ptr,
                                                                     const ORUtils::SE3Pose *pose) {
        sceneIsBackground = obj_inst_ptr->CheckIsBackground();
        auto *scene = obj_inst_ptr->GetScene().get();
        visualisationEngine->FindVisibleBlocks(scene,
                                               pose,
                                               &obj_inst_ptr->GetCurrentView()->calib.intrinsics_d,
                                               obj_inst_ptr->GetRenderState().get());

        visualisationEngine->CreateExpectedDepths(scene,
                                                  pose,
                                                  &obj_inst_ptr->GetCurrentView()->calib.intrinsics_d,
                                                  obj_inst_ptr->GetRenderState().get());
        obj_inst_ptr->UpdateVisibility();

    }


    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::UpdateVisibilityOfAllObj() {
        if (obj_inst_ptr_vector.size() == 0) return;
        sceneIsBackground = false;

#pragma omp parallel for private(sceneIsBackground)
        for (size_t i = 1; i < this->obj_inst_ptr_vector.size(); ++i) { //start from1 to skip BG

            const ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr = obj_inst_ptr_vector.at(i);
            bool prevVisibility = obj_inst_ptr->isVisible;

            UpdateVisibilityOfObj(obj_inst_ptr, this->t_state->pose_d);

#pragma omp critical
            {
                if (prevVisibility && !(obj_inst_ptr->isVisible)) {
                    active_obj_ptr_vector.erase(
                            std::remove(active_obj_ptr_vector.begin(), active_obj_ptr_vector.end(), obj_inst_ptr),
                            active_obj_ptr_vector.end());
                }
                if (!prevVisibility && obj_inst_ptr->isVisible) {
                    active_obj_ptr_vector.push_back(obj_inst_ptr);
                }
            }
        }
//        active_obj_ptr_vector.shrink_to_fit();
        this->number_totalObjects = obj_inst_ptr_vector.size();
        this->number_activeObjects = active_obj_ptr_vector.size();

//        BG_VoxelCleanUp();
    }

//check if same obj by 2d overlap
    template<class TVoxel, class TIndex>
    bool ObjSLAMMappingEngine<TVoxel, TIndex>::CheckIsSameObject2D(ObjectInstance_ptr <TVoxel, TIndex> obj_ptr_1,
                                                                   ObjectInstance_ptr <TVoxel, TIndex> obj_ptr_2) {
//  cout<<"CheckIsSameObject2D\n";
        bool isSame = false;
        ObjSLAM::ObjFloatImage *first = obj_ptr_1.get()->GetAnchorView_ITM()->depth;
        ObjSLAM::ObjFloatImage *second = obj_ptr_2.get()->GetAnchorView_ITM()->depth;

        auto *firstUchar4 = ProjectObjectToImg(obj_ptr_1);

        auto *cam = new ObjSLAMCamera(this->calib, this->imgSize);

        auto *pcl = new ORUtils::Image<Vector4f>(imgSize, MEMORYDEVICE_CPU);//in world coordinate
        //TODO change pose to the pose from obj anchor view. add pose to itm view or let the obj inst save the pose itself.
        cam->ProjectImg2PointCloud(second, pcl, obj_ptr_2->GetAnchorView()->GetCameraPose().GetSE3Pose());
        //  cout<<*this->t_state->pose_d;
//TODO segfault

        ObjFloatImage *out = new ObjFloatImage(imgSize, MEMORYDEVICE_CPU);
        cam->ProjectPointCloud2Img(pcl, out, obj_ptr_1->GetAnchorView()->GetCameraPose().GetSE3Pose());

        delete cam;
        delete pcl;
        return CheckImageOverlap(firstUchar4, second) || CheckImageOverlap(first, out);
//  return CheckImageOverlap(first, out);
    }

    template<class TVoxel, class TIndex>
    ORUtils::Image<Vector4u> *ObjSLAMMappingEngine<TVoxel,
            TIndex>::ProjectObjectToImg(ObjectInstance_ptr <TVoxel, TIndex> obj_inst_ptr) {

        sceneIsBackground = obj_inst_ptr->CheckIsBackground();

        const auto scene = obj_inst_ptr->GetScene();

        visualisationEngine->RenderImage(scene.get(),
                                         this->t_state->pose_d,
                                         &obj_inst_ptr.get()->GetAnchorView_ITM()->calib.intrinsics_d,
                                         obj_inst_ptr->GetRenderState().get(),
                                         obj_inst_ptr->GetRenderState()->raycastImage,
                                         ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_COLOUR_FROM_VOLUME,
                                         ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_FROM_NEW_RAYCAST);

        return obj_inst_ptr->GetRenderState()->raycastImage;
    }

    template<class TVoxel, class TIndex>
    ORUtils::Image<Vector4f> *ObjSLAMMappingEngine<TVoxel,
            TIndex>::ProjectObjectToFloatImg(ObjectInstance_ptr <TVoxel, TIndex> obj_inst_ptr) {

        sceneIsBackground = obj_inst_ptr->CheckIsBackground();

        auto scene = obj_inst_ptr->GetScene();

        visualisationEngine->RenderImage(scene.get(),
                                         this->t_state->pose_d,
                                         &obj_inst_ptr.get()->GetAnchorView_ITM()->calib.intrinsics_d,
                                         obj_inst_ptr->GetRenderState().get(),
                                         obj_inst_ptr->GetRenderState()->raycastImage,
                                         ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_COLOUR_FROM_VOLUME,
                                         ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_FROM_NEW_RAYCAST);
        return obj_inst_ptr->GetRenderState()->raycastResult;
    }

    template<class TVoxel, class TIndex>
    bool ObjSLAMMappingEngine<TVoxel, TIndex>::CheckImageOverlap(ObjSLAM::ObjFloatImage *first,
                                                                 ObjSLAM::ObjFloatImage *second) {
//  cout<<"CheckImageOverlap\n";
        //parameter to set which % of the pixels must match
        double threshold_areaChange = 0.05;
        double threshold_overlap = 0.2;

        int x1_min = imgSize.x - 1;
        int x2_min = imgSize.x - 1;
        int x1_max = 0;
        int x2_max = 0;

        int y1_min = imgSize.y - 1;
        int y2_min = imgSize.y - 1;
        int y1_max = 0;
        int y2_max = 0;

//not parallel due to data race
        for (size_t j = 0; j < imgSize.y; ++j) {
            for (size_t i = 0; i < imgSize.x; ++i) {
                int idx = j * imgSize.x + i;
                if (first->GetElement(idx, MEMORYDEVICE_CPU) > 0) {
                    if (i < x1_min) x1_min = i;
                    if (i > x1_max) x1_max = i;
                    if (j < y1_min) y1_min = j;
                    if (j > y1_max) y1_max = j;
                }
                if (second->GetElement(idx, MEMORYDEVICE_CPU) > 0) {
                    if (i < x2_min) x2_min = i;
                    if (i > x2_max) x2_max = i;
                    if (j < y2_min) y2_min = j;
                    if (j > y2_max) y2_max = j;
                }
            }
        }

        if (x1_min > x2_max || x2_min > x1_max || y1_min > y2_max || y2_min > y1_max) return false;

        float area_1 = (x1_max - x1_min) * (y1_max - y1_min);
        float area_2 = (x2_max - x2_min) * (y2_max - y2_min);

        int x_min_overlap = MAX(x1_min, x2_min);
        int y_min_overlap = MAX(y1_min, y2_min);
        int x_max_overlap = MIN(x1_max, x2_max);
        int y_max_overlap = MIN(y1_max, y2_max);

        float area_overlap = (x_max_overlap - x_min_overlap) * (y_max_overlap - y_min_overlap);

//  cout << "check" << min(area_1, area_2) / max(area_1, area_2) << " " << area_overlap / min(area_1, area_2) << endl;

        if (area_1 <= area_2) {
            return area_1 / area_2 > threshold_areaChange && area_overlap / area_1 > threshold_overlap;
        } else {
            return area_2 / area_1 > threshold_areaChange && area_overlap / area_2 > threshold_overlap;
        }
    }

    template<class TVoxel, class TIndex>
    bool ObjSLAMMappingEngine<TVoxel, TIndex>::CheckImageOverlap(ORUtils::Image<Vector4u> *first,
                                                                 ObjSLAM::ObjFloatImage *second) {
//  cout<<"CheckImageOverlap\n";
        //parameter to set which % of the pixels must match
        double threshold_areaChange = 0.05;
        double threshold_overlap = 0.25;

        int x1_min = imgSize.x - 1;
        int x2_min = imgSize.x - 1;
        int x1_max = 0;
        int x2_max = 0;

        int y1_min = imgSize.y - 1;
        int y2_min = imgSize.y - 1;
        int y1_max = 0;
        int y2_max = 0;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
        for (size_t j = 0; j < imgSize.y; ++j) {
            for (size_t i = 0; i < imgSize.x; ++i) {
                int idx = j * imgSize.x + i;
                auto uchar4ImgPixel = first->GetElement(idx, MEMORYDEVICE_CPU);
                if (uchar4ImgPixel.x != 0 || uchar4ImgPixel.y != 0 || uchar4ImgPixel.z != 0) {
                    if (i < x1_min) x1_min = i;
                    if (i > x1_max) x1_max = i;
                    if (j < y1_min) y1_min = j;
                    if (j > y1_max) y1_max = j;
                }
                if (second->GetElement(idx, MEMORYDEVICE_CPU) > 0) {
                    if (i < x2_min) x2_min = i;
                    if (i > x2_max) x2_max = i;
                    if (j < y2_min) y2_min = j;
                    if (j > y2_max) y2_max = j;
                }
            }
        }


        if (x1_min > x2_max || x2_min > x1_max || y1_min > y2_max || y2_min > y1_max) return false;

        float area_1 = (x1_max - x1_min) * (y1_max - y1_min);
        float area_2 = (x2_max - x2_min) * (y2_max - y2_min);

        int x_min_overlap = max(x1_min, x2_min);
        int y_min_overlap = max(y1_min, y2_min);
        int x_max_overlap = min(x1_max, x2_max);
        int y_max_overlap = min(y1_max, y2_max);

        float area_overlap = (x_max_overlap - x_min_overlap) * (y_max_overlap - y_min_overlap);

//  cout << "check" << min(area_1, area_2) / max(area_1, area_2) << " " << area_overlap / min(area_1, area_2) << endl;

        if (area_1 <= area_2) {
            return area_1 / area_2 > threshold_areaChange && area_overlap / area_1 > threshold_overlap;
        } else {
            return area_2 / area_1 > threshold_areaChange && area_overlap / area_2 > threshold_overlap;
        }
    }

//check if same obj by 3d overlap
    template<class TVoxel, class TIndex>
    bool ObjSLAMMappingEngine<TVoxel, TIndex>::CheckIsSameObject(ObjectInstance_ptr <TVoxel, TIndex> obj_ptr_1,
                                                                 ObjectInstance_ptr <TVoxel, TIndex> obj_ptr_2) {

        ObjSLAM::ObjFloatImage *first = obj_ptr_1.get()->GetAnchorView_ITM()->depth;
        ObjSLAM::ObjFloatImage *second = obj_ptr_2.get()->GetAnchorView_ITM()->depth;

        auto *cam = new ObjSLAMCamera(this->calib, this->imgSize);

        auto *pcl1 = new ORUtils::Image<Vector4f>(imgSize, MEMORYDEVICE_CPU);//in world coordinate
        auto *pcl2 = new ORUtils::Image<Vector4f>(imgSize, MEMORYDEVICE_CPU);//in world coordinate

        ORUtils::Vector6<float> cube1 =
                cam->ProjectImg2PointCloud(first, pcl1,
                                           obj_ptr_1.get()->GetAnchorView().get()->GetCameraPose().GetSE3Pose());
        ORUtils::Vector6<float> cube2 = cam->ProjectImg2PointCloud(second, pcl2, *this->t_state->pose_d);

        delete cam;
        delete pcl1;
        delete pcl2;

        return CheckBoundingCubeOverlap(cube1, cube2);
    }

//using overlapping volumes.
    template<class TVoxel, class TIndex>
    bool ObjSLAMMappingEngine<TVoxel, TIndex>::CheckBoundingCubeOverlap(ORUtils::Vector6<float> first,
                                                                        ORUtils::Vector6<float> second) {

        double threshold_volumeChange = 0.4;
        double threshold_overlap = 0.6;

        //case where there is 0 overlap
        if (second[0] > first[3] || second[1] > first[4] || second[2] > first[5] || first[0] > second[3]
            || first[1] > second[4] || first[2] > second[5])
            return false;

        double max_of_min_x = max(first[0], second[0]);
        double max_of_min_y = max(first[1], second[1]);
        double max_of_min_z = max(first[2], second[2]);

        double min_of_max_x = min(first[3], second[3]);
        double min_of_max_y = min(first[4], second[4]);
        double min_of_max_z = min(first[5], second[5]);

        ORUtils::Vector6<float> overlap(max_of_min_x, max_of_min_y, max_of_min_z, min_of_max_x, min_of_max_y,
                                        min_of_max_z);

        double v1 = CalculateCubeVolume(first);
        double v2 = CalculateCubeVolume(second);
        double v_overlap = CalculateCubeVolume(overlap);

//  cout << "check" << min(v1, v2) / max(v1, v2) << " " << v_overlap / min(v1, v2) << endl;

        if (v1 <= v2) {
            return v1 / v2 > threshold_volumeChange && v_overlap / v1 > threshold_overlap;
        } else {
            return v2 / v1 > threshold_volumeChange && v_overlap / v2 > threshold_overlap;
        }
    }


    template<class TVoxel, class TIndex>
    double ObjSLAMMappingEngine<TVoxel, TIndex>::CalculateCubeVolume(ORUtils::Vector6<float> corners) {
        double l = (corners[3] - corners[0]);
        double w = (corners[4] - corners[1]);
        double h = (corners[5] - corners[2]);
        return l * w * h;
    }

    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::UpdateTrackingState(const ORUtils::SE3Pose *_pose) {
        if (this->t_state == NULL) {
            this->t_state = new ITMLib::ITMTrackingState(imgSize, MEMORYDEVICE_CPU);
        }
        this->t_state->pose_d->SetFrom(_pose);
        this->t_state->pose_d->Coerce();

        this->UpdateViewPose();
    }

    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::UpdateTrackingState(shared_ptr<ITMLib::ITMTrackingState> _t_state) {
        this->t_state = _t_state;
        UpdateVisibilityOfAllObj();
//  this->UpdateViewPose();
    }

    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::UpdateViewPose() {
        ObjCameraPose _pose(*(this->t_state->pose_d));
        this->view->SetCameraPose(_pose);
    }

    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::SetTrackingController(
            shared_ptr<ITMLib::ITMTrackingController> _t_controller) {
        t_controller = _t_controller;
    }

    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::DeleteAll() {
        delete this->visualisationEngine;
        delete this->visualisationEngine_BG;
        delete this->denseMapper;
//        delete[] preAllocation_array;
    }


    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::SaveSceneToMesh(const char *objFileName,
                                                               std::shared_ptr<ITMLib::ITMScene<TVoxel,
                                                                       TIndex>> scene_ptr) {
        auto meshingEngine = ITMLib::ITMMeshingEngineFactory::MakeMeshingEngine<TVoxel, TIndex>(settings->deviceType);

        ITMLib::ITMMesh *this_mesh = new ITMLib::ITMMesh(settings->GetMemoryType());

        meshingEngine->MeshScene(this_mesh, scene_ptr.get());
        this_mesh->WriteSTL(objFileName);

        delete this_mesh;
    }

    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::write2PLYfile(const ORUtils::Image<ORUtils::Vector4<float>> *pcl,
                                                             const std::string filename) {
        std::ofstream fileWriter;
        fileWriter.open(filename.c_str(), std::ofstream::out);
        if (!fileWriter.is_open()) {
            std::cout << "Cannot open file for PLY file writing " << filename.c_str() << "\n";
        }
        std::cout << "Point cloud is saved to " << filename.c_str() << "\n";

        // write header
        fileWriter << "ply\n";
        fileWriter << "format ascii 1.0\n";
        fileWriter << "element vertex " << pcl->dataSize << "\n";
        fileWriter << "property float32 x\n";
        fileWriter << "property float32 y\n";
        fileWriter << "property float32 z\n";
        fileWriter << "end_header\n";

        for (int i = 0; i < pcl->dataSize; i++) {
            ORUtils::Vector4<float> point = pcl->GetElement(i, MEMORYDEVICE_CPU);
            fileWriter << point.x << " " << point.y << " " << point.z << "\n";
        }

        fileWriter.close();
    }


    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::Object_Cleanup(ObjectInstance_ptr <TVoxel, TIndex> object) {
        sceneIsBackground = object->CheckIsBackground();
        float threshold = 0.8;
        short k_weight = 4;
        float th_weight = 0.5;
        short k_minAge = 2;

        auto scene = object->GetScene();
//        short object_view_count = object->view_count;

        //stuffs varry over from ResetScene
        int numBlocks = scene->index.getNumAllocatedVoxelBlocks();
        int blockSize = scene->index.getVoxelBlockSize();
        auto *renderState_vh = (ITMLib::ITMRenderState_VH *) object->GetRenderState().get();
        int *visibleEntryIds = renderState_vh->GetVisibleEntryIDs();
        int noVisibleEntries = renderState_vh->noVisibleEntries;

        TVoxel *voxelBlocks_ptr = scene->localVBA.GetVoxelBlocks();
        ITMHashEntry *hashTable = scene->index.GetEntries();
        TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
//        for (int entryId = 0; entryId < noVisibleEntries; entryId++) {
        for (int i = 0; i < (sceneIsBackground ? scene->index.noTotalEntries_BG : scene->index.noTotalEntries); ++i) {
//            float abs_weight_th = th_weight / MIN(settings->sceneParams.maxW, object_view_count);

            const ITMHashEntry &currentHashEntry = hashTable[i];

            if (currentHashEntry.ptr < 0) continue;


            TVoxel *localVoxelBlock = &(localVBA[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);

            for (int locId = 0; locId < SDF_BLOCK_SIZE3; locId++) {

                int w = localVoxelBlock[locId].w_depth;
                int view_count = localVoxelBlock[locId].view_count;
                int alloc_count = localVoxelBlock[locId].alloc_count;
//                std::cout<<"weight "<<w<<" view count "<<view_count<<" alloc count "<<alloc_count<<std::endl;
                if (alloc_count > k_minAge && alloc_count <= th_weight * view_count) {
                    //remove this voxel
                    localVoxelBlock[locId] = TVoxel();
                }
            }
        }
    }


    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::BG_VoxelCleanUp() {

        std::vector<Vector3s> voxelPos_vec;
        voxelPos_vec.reserve(100000);

//TODO Parallelize this loop
/*#ifdef WITH_OPENMP
#pragma omp parallel
#endif
  std::vector<Vector3f> voxelPos_vec_private;
#ifdef WITH_OPENMP
#pragma omp for
#endif */
        for (size_t i = 0; i < this->obj_inst_ptr_vector.size(); ++i) {
            ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr = obj_inst_ptr_vector.at(i);
            if (!obj_inst_ptr->CheckIsBackground()) GetVoxelPosFromScene(voxelPos_vec, obj_inst_ptr);
        }

        int size = voxelPos_vec.size();

        ITMLib::ITMScene<TVoxel, TIndex> *scene_BG = this->BG_object_ptr->GetScene().get();
        sceneIsBackground = true;
        for (size_t i = 0; i < size; ++i) {
            Vector3s blockPos = voxelPos_vec.at(i);
            int hashIdx = hashIndex(blockPos);
            TVoxel *voxelData = scene_BG->localVBA.GetVoxelBlocks();
            int *voxelAllocationList = scene_BG->localVBA.GetAllocationList();
            int noAllocatedVoxelEntries = scene_BG->localVBA.lastFreeBlockId;

            //loop in the hashBucket
            while (true) {
                ITMHashEntry hashEntry = scene_BG->index.GetEntries()[hashIdx];

                if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= 0) {


                    int blockIdx = hashEntry.ptr * SDF_BLOCK_SIZE3;
                    if (hashEntry.ptr >= 0) {

                        //copied from SwappingEngine_CPU
                        int vbaIdx = noAllocatedVoxelEntries;
                        if (vbaIdx < (sceneIsBackground ? SDF_BUCKET_NUM_BG : SDF_BUCKET_NUM) - 1) {
                            noAllocatedVoxelEntries++;
                            voxelAllocationList[vbaIdx + 1] = hashEntry.ptr;
                            hashEntry.ptr = -1;

                            for (int idx = 0; idx < SDF_BLOCK_SIZE3; idx++) voxelData[blockIdx + idx] = TVoxel();
                        }
                    }

                }

                if (hashEntry.offset < 1) break;
                hashIdx = (sceneIsBackground ? SDF_BUCKET_NUM_BG : SDF_BUCKET_NUM) + hashEntry.offset - 1;
            }
        }
    }


    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::
    GetVoxelPosFromScene(std::vector<Vector3s> &voxelPos_vec, ObjectInstance_ptr <TVoxel, TIndex> obj_ptr) {

        ITMLib::ITMScene<TVoxel, TIndex> *scene = obj_ptr->GetScene().get();
//  TIndex index = scene->index;
//  ITMLib::ITMLocalVBA<TVoxel> locVBA = scene->localVBA;
        ITMLib::ITMRenderState_VH *renderState_vh = (ITMLib::ITMRenderState_VH *) obj_ptr->GetRenderState().get();
        int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();
        const typename ITMLib::ITMVoxelBlockHash::IndexData *voxelIndex = scene->index.getIndexData();

        //see how to read excess, maybe use readvoxel
        for (int blockNo = 0; blockNo < renderState_vh->noVisibleEntries; ++blockNo) {
            int blockID = visibleEntryIDs[blockNo];
            ITMHashEntry &blockData(scene->index.GetEntries()[blockID]);
            voxelPos_vec.push_back(blockData.pos); //TODO here sometimes segfault due to blockdata being empty
//    cout << "BlockNo" << blockNo << " Pos" << blockData.pos << endl;
        }
    }

    template<class TVoxel, class TIndex>
    ObjUChar4Image *ObjSLAMMappingEngine<TVoxel, TIndex>::GetImage(int object_index) {
        if (object_index == 0) return GetImage(BG_object_ptr);

        if (object_index < number_activeObjects) {
            return GetImage(active_obj_ptr_vector.at(object_index));
        } else {
            cout << "Object Index larger than total number of objects, showing first object...\n";
            return GetImage(BG_object_ptr);
        }
    }


    template<class TVoxel, class TIndex>
    ObjUChar4Image *ObjSLAMMappingEngine<TVoxel, TIndex>::GetImage(ObjectInstance_ptr <TVoxel, TIndex> obj_inst_ptr) {
        return obj_inst_ptr->GetRenderState()->raycastImage;
    }


    template<class TVoxel, class TIndex>
    ObjUChar4Image *ObjSLAMMappingEngine<TVoxel, TIndex>::GetBGImage() {
        return this->img_BG.get();
    }

    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::RenderImageFromAbove() {
        sceneIsBackground = true;

        this->img_above = std::make_shared<ObjUChar4Image>(imgSize, MEMORYDEVICE_CPU);

        auto *pose_visualize = this->t_state_above->pose_d;

        renderState_RenderAbove->raycastImage->Clear();

        //Insert function: prepare tracking with all objs
        t_controller->Prepare(t_state_above.get(), renderState_RenderAbove.get(), this->obj_inst_ptr_vector,
                              visualisationEngine_BG);

        img_above->ChangeDims(BG_object_ptr->GetRenderState().get()->raycastImage->noDims);


#ifdef WITH_OPENMP
#pragma omp parallel for private(sceneIsBackground)
#endif
        for (int i = 0; i < obj_inst_ptr_vector.size(); ++i) {
            auto obj_inst_ptr = obj_inst_ptr_vector.at(i);
            sceneIsBackground = obj_inst_ptr->CheckIsBackground();
            auto scene = obj_inst_ptr->GetScene();


            visualisationEngine_BG->FindVisibleBlocks(scene.get(), pose_visualize,
                                                      &obj_inst_ptr->GetCurrentView()->calib.intrinsics_d,
                                                      obj_inst_ptr->GetRenderStateAbove().get());

            visualisationEngine_BG->CreateExpectedDepths(scene.get(), pose_visualize,
                                                         &obj_inst_ptr->GetCurrentView()->calib.intrinsics_d,
                                                         obj_inst_ptr->GetRenderStateAbove().get());


            visualisationEngine_BG->RenderImage(scene.get(), pose_visualize,
                                                &BG_object_ptr.get()->GetAnchorView_ITM()->calib.intrinsics_d,
                                                obj_inst_ptr->GetRenderStateAbove().get(),
                                                obj_inst_ptr->GetRenderStateAbove()->raycastImage,
                                                ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_COLOUR_FROM_VOLUME,
                                                ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_FROM_NEW_RAYCAST);
        }

        for (int i = 0; i < obj_inst_ptr_vector.size(); ++i) {
#ifdef WITH_OPENMP
#pragma omp parallel
#endif
            for (int idx = 0; idx < img_above->dataSize; idx++) {
                ITMLib::ITMRenderState *r_state_abv = obj_inst_ptr_vector.at(i)->GetRenderStateAbove().get();
                Vector4u pixel = img_above->GetData(MEMORYDEVICE_CPU)[idx];
                Vector4u pixel_obj = r_state_abv->raycastImage->GetData(MEMORYDEVICE_CPU)[idx];
                if (pixel.r == 0 && pixel.g == 0 && pixel.b == 0)
                    img_above->GetData(MEMORYDEVICE_CPU)[idx] += pixel_obj;
//                img_above->GetData(MEMORYDEVICE_CPU)[idx] += r_state_abv->raycastImage->GetData(MEMORYDEVICE_CPU)[idx];
            }

        }

        SaveImageToFile(img_above.get(), ("BG_Above" + to_string(imgNumber) + ".ppm").c_str());
    }

    template<class TVoxel, class TIndex>
    ObjUChar4Image *ObjSLAMMappingEngine<TVoxel, TIndex>::GetImageFromAbove() {
        return this->img_above.get();
    }


    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::RenderAllObjImages() {

        cout << "Number of Objects = " << number_totalObjects << endl;
        cout << "Number of visible Objects = " << number_activeObjects << endl;

        auto *pose_visualize = this->t_state->pose_d;
        img_BG->ChangeDims(this->renderState_RenderAll->raycastImage->noDims);
        img_BG->Clear();
        this->renderState_RenderAll->raycastImage->Clear();

#ifdef WITH_OPENMP
#pragma omp parallel for private(sceneIsBackground)
#endif
        for (size_t i = 0; i < this->label_ptr_vector.size(); ++i) {
            std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>>
                    label_ptr = label_ptr_vector.at(i);
            std::vector<ObjectInstance_ptr<TVoxel, TIndex>> &
                    obj_inst_vec = (label_ptr.get()->GetObjPtrVector());
            cout << *label_ptr.get() << " : " << obj_inst_vec.size() << endl;
#ifdef WITH_OPENMP
#pragma omp parallel for private(sceneIsBackground)
#endif
            for (size_t j = 0; j < obj_inst_vec.size(); ++j) {
                ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr = obj_inst_vec.at(j);

                auto scene = obj_inst_ptr->GetScene();

                obj_inst_ptr->GetRenderState()->raycastImage->Clear();
                obj_inst_ptr->GetRenderState()->raycastResult->Clear();

                if (obj_inst_ptr->GetLabelIndex() != 0) {
                    sceneIsBackground = obj_inst_ptr->CheckIsBackground();

//                    obj_inst_ptr->GetRenderState()->raycastImage->Clear();


                    UpdateVisibilityOfObj(obj_inst_ptr, pose_visualize);


                    if (!obj_inst_ptr->isVisible) {
                        auto scene = obj_inst_ptr.get()->GetScene();
                        string stlname = obj_inst_ptr->GetClassLabel()->GetLabelClassName() + "." + to_string(j) +
                                         "_cleaned.stl";
                        SaveSceneToMesh(stlname.c_str(), scene);
                        continue;
                    }


                    visualisationEngine->RenderImage(scene.get(), pose_visualize,
                                                     &obj_inst_ptr->GetAnchorView_ITM()->calib.intrinsics_d,
                                                     obj_inst_ptr->GetRenderState().get(),
                                                     obj_inst_ptr->GetRenderState()->raycastImage,
                                                     ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_COLOUR_FROM_VOLUME,
                                                     ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_FROM_NEW_RAYCAST);

                    visualisationEngine->RenderImage(scene.get(), pose_visualize,
                                                     &obj_inst_ptr->GetAnchorView_ITM()->calib.intrinsics_d,
                                                     this->renderState_RenderAll.get(),
                                                     this->renderState_RenderAll->raycastImage,
                                                     ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_COLOUR_FROM_VOLUME,
                                                     ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_FROM_OLD_RAYCAST);

                } else {
                    sceneIsBackground = true;

//update visibility? TODO

                    visualisationEngine->RenderImage(scene.get(), pose_visualize,
                                                     &BG_object_ptr->GetAnchorView_ITM()->calib.intrinsics_d,
                                                     BG_object_ptr->GetRenderState().get(),
                                                     BG_object_ptr->GetRenderState()->raycastImage,
                                                     ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_COLOUR_FROM_VOLUME,
                                                     ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_FROM_NEW_RAYCAST);
                    visualisationEngine->RenderImage(scene.get(), pose_visualize,
                                                     &BG_object_ptr->GetAnchorView_ITM()->calib.intrinsics_d,
                                                     this->renderState_RenderAll.get(),
                                                     this->renderState_RenderAll->raycastImage,
                                                     ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_COLOUR_FROM_VOLUME,
                                                     ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_FROM_OLD_RAYCAST);

                    write2PLYfile(obj_inst_ptr->GetRenderState()->raycastResult,
                                  "raycast_img_BG" + to_string(imgNumber) + ".ply");
                }


//uncomment this for pixel wise fusion of rendering
/*//possible data race due to outer loop?
#ifdef WITH_OPENMP
#pragma omp parallel
#endif
                for (int idx = 0; idx < img_BG->dataSize; idx++) {
                    ITMLib::ITMRenderState *r_state_obj = obj_inst_ptr->GetRenderState().get();
                    Vector4u pixel = img_BG->GetData(MEMORYDEVICE_CPU)[idx];
                    Vector4u pixel_obj = r_state_obj->raycastImage->GetData(MEMORYDEVICE_CPU)[idx];
                    if (pixel.r == 0 && pixel.g == 0 && pixel.b == 0)
                        img_BG->GetData(MEMORYDEVICE_CPU)[idx] += pixel_obj;
                }


                //*/


            }//end inner obj loop

        }//end outer label loop

        /*visualisationEngine_BG->RenderImageMulti(obj_inst_ptr_vector, pose_visualize,
                                                 &BG_object_ptr->GetAnchorView_ITM()->calib.intrinsics_d,
                                                 this->renderState_RenderAll.get(),
                                                 this->renderState_RenderAll->raycastImage,
                                                 ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_COLOUR_FROM_VOLUME,
                                                 ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_FROM_NEW_RAYCAST);
*/
        write2PLYfile(this->renderState_RenderAll->raycastResult, "raycast_img_ALL" + to_string(imgNumber) + ".ply");
        sceneIsBackground = true;

        img_BG->SetFrom(this->renderState_RenderAll->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
        string name_BG = "Label_BG_Fused.Object0.Frame" +
                         to_string(imgNumber)
                         + ".ppm";

        SaveImageToFile(img_BG.get(), name_BG.c_str());


        RenderImageFromAbove();

        //save stl
        if (saveSTL && imgNumber % STL_Frequency == 0) {
//#pragma omp parallel for private(sceneIsBackground)
            for (size_t i = 0; i < this->obj_inst_ptr_vector.size(); ++i) {
                ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr = obj_inst_ptr_vector.at(i);
                sceneIsBackground = obj_inst_ptr->CheckIsBackground();
                auto scene = obj_inst_ptr.get()->GetScene();
                string stlname = obj_inst_ptr->GetClassLabel()->GetLabelClassName() + "." + to_string(i) + ".stl";
                SaveSceneToMesh(stlname.c_str(), scene);
            }
        }
    }

    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::OutputAllObjImages() {
#ifdef WITH_OPENMP
#pragma omp parallel for private(sceneIsBackground)
#endif
        for (size_t i = 0; i < this->label_ptr_vector.size(); ++i) {
            std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>>
                    label_ptr = label_ptr_vector.at(i);
            std::vector<ObjectInstance_ptr<TVoxel, TIndex>>
                    &
                    obj_inst_vec = (label_ptr.get()->GetObjPtrVector());

#ifdef WITH_OPENMP
#pragma omp parallel for private(sceneIsBackground)
#endif
            for (size_t j = 0; j < obj_inst_vec.size(); ++j) {
                ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr = obj_inst_vec.at(j);
                string name =
                        "Label_" + label_ptr.get()->GetLabelClassName() + ".Object" + to_string(j) + ".Frame" +
                        to_string(imgNumber)
                        + ".ppm";

                SaveImageToFile(obj_inst_ptr->GetRenderState()->raycastImage, name.c_str());
            }

        }
    }
}

