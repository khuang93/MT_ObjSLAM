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
        view->setListOfObjects(label_ptr_vector);

    }

    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::UpdateObjBoolImg() {
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
        for (size_t i = 0; i < obj_inst_ptr_vector.size(); ++i) {
            const ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr = obj_inst_ptr_vector.at(i);

            auto *img = this->projectObjectToImg(obj_inst_ptr);
            std::shared_ptr<ObjBoolImage> boolImg = std::make_shared<ObjBoolImage>(imgSize, true, false);
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
            for (size_t it = 0; it < img->dataSize; ++it) {
                if (img->GetData(MEMORYDEVICE_CPU)[it].x != 0 || img->GetData(MEMORYDEVICE_CPU)[it].y != 0
                    || img->GetData(MEMORYDEVICE_CPU)[it].z != 0) {
                    boolImg->GetData(MEMORYDEVICE_CPU)[it] = true;
                }
            }
//    SaveImageToFile(boolImg.get(), "Projected_bool.ppm");
            obj_inst_ptr->setBoolImage(boolImg);
        }
    }

    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::ApplyBoolImg(ObjectInstance_ptr <TVoxel, TIndex> BGobj,
                                                            shared_ptr<ObjBoolImage> boolImg) {
        shared_ptr<ITMLib::ITMView> BG_view = BGobj->getCurrentView();
        ITMFloatImage *depthImg = BG_view->depth;
//  SaveImageToFile(depthImg,("DepthIn_Before_"+to_string(imgNumber)+".pgm").c_str());

        for (size_t i = 0; i < depthImg->dataSize; ++i) {
            depthImg->GetData(MEMORYDEVICE_CPU)[i] =
                    (int) (!(boolImg->GetData(MEMORYDEVICE_CPU)[i])) * depthImg->GetData(MEMORYDEVICE_CPU)[i];
        }
//  SaveImageToFile(depthImg,("DepthIn_Aft_"+to_string(imgNumber)+".pgm").c_str());
    }

    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::ProcessFrame() {

        bool useSwapping = (settings->swappingMode != ITMLib::ITMLibSettings::SWAPPINGMODE_DISABLED);

        cout << "ProcessFrame...\n";

        if (view->getObjVec().size() > 0) {

            bool newObject = true;

            int number_free_preallocation = false; // this->number_preallocation - number_totalObjects;

            bool usePreallocation = number_free_preallocation > view->getObjVec().size();
#ifdef WITH_OPENMP
#pragma omp parallel for private(sceneIsBackground) /*num_threads(numthreads)*/
#endif
            for (int t = 0; t < view->getObjVec().size(); t++) {

                Object_View_Tup<TVoxel, TIndex> view_tuple = view->getObjVec().at(t);

                auto obj_inst_ptr = std::get<0>(view_tuple);
                auto label_ptr = obj_inst_ptr->getClassLabel();

                int labelIndex = label_ptr->getLabelIndex();

                if (labelIndex == 74) continue; //skip books

                //set the flag of background or not
                sceneIsBackground = labelIndex == 0 ? true : false;
//                cout<<"sceneIsBackground "<<sceneIsBackground<<endl;

                std::shared_ptr<ITMLib::ITMView> itmview = std::get<1>(view_tuple);

                auto &obj_ptr_vec_val = label_ptr->getObjPtrVector();
                auto obj_ptr_vec = &obj_ptr_vec_val;


                if (obj_ptr_vec->size() == 0) {
                    obj_inst_ptr->addObjectInstanceToLabel();
                    if (obj_inst_ptr->checkIsBackground()) {
                        obj_inst_ptr_vector.insert(obj_inst_ptr_vector.begin(), obj_inst_ptr);
                    } else {
                        obj_inst_ptr_vector.push_back(obj_inst_ptr);
                    }
                    newObject = true;
                } else {
//#ifdef WITH_OPENMP
//#pragma omp parallel for private(sceneIsBackground) /*num_threads(numthreads)*/
//#endif
                    for (size_t i = 0; i < obj_ptr_vec->size(); ++i) {
                        ObjectInstance_ptr<TVoxel, TIndex> existing_obj_ptr = obj_ptr_vec->at(i);

                        if (obj_inst_ptr->checkIsBackground()) {
                            newObject = false;
                        } else if (existing_obj_ptr->isVisible) {
                            sceneIsBackground == false;
                            newObject = !this->checkIsSameObject2D(existing_obj_ptr, obj_inst_ptr);
//                            cout<<"sceneIsBackground "<<sceneIsBackground<<endl;
                        }
                        if (!newObject) {
                            //this is an existing object, no need to compare with further objs
//            obj_inst_scene_ptr = existing_obj_ptr->getScene();

                            obj_inst_ptr = existing_obj_ptr;

                            break;
                        }
                    }
                    if (newObject) {
                        obj_inst_ptr->addObjectInstanceToLabel();
                        obj_inst_ptr_vector.push_back(obj_inst_ptr);
                    }
                }


                obj_inst_ptr->setCurrentView(itmview);

                if (newObject) {
#pragma omp atomic
                    number_totalObjects++;
//                    number_activeObjects++;
                    sceneIsBackground = obj_inst_ptr->checkIsBackground();
                    obj_inst_ptr->view_count=1;
                    if (obj_inst_ptr->checkIsBackground() && BG_object_ptr.get() == nullptr) {
                        BG_object_ptr = obj_inst_ptr;
                    }


                    if (!usePreallocation) {
                        auto obj_inst_scene_ptr = std::make_shared<ObjectInstanceScene<TVoxel, TIndex>>(
                                sceneParams_ptr.get(),
                                useSwapping,
                                MEMORYDEVICE_CPU);
                        obj_inst_ptr->setScene(obj_inst_scene_ptr);
                        denseMapper->ResetScene(obj_inst_ptr->getScene().get());
                    } else {
//                        obj_inst_ptr->setScene(preAllocation_array[number_totalObjects]);
                    }


                    std::shared_ptr<ITMLib::ITMRenderState> renderState_ptr(
                            new ITMLib::ITMRenderState_VH((sceneIsBackground
                                                           ? ITMLib::ITMVoxelBlockHash::noTotalEntries_BG
                                                           : ITMLib::ITMVoxelBlockHash::noTotalEntries),
                                                          imgSize,
                                                          settings->sceneParams.viewFrustum_min,
                                                          settings->sceneParams.viewFrustum_max,
                                                          MEMORYDEVICE_CPU));
                    obj_inst_ptr->setRenderState(renderState_ptr);

                    auto t_state_ptr = std::make_shared<ITMLib::ITMTrackingState>(imgSize, MEMORYDEVICE_CPU);
                    obj_inst_ptr->setTrackingState(t_state_ptr);

//                    denseMapper->ResetScene(obj_inst_scene_ptr.get());

                } else {
                    obj_inst_ptr->view_count++;
                    cout << "obj_view_count = " << obj_inst_ptr->view_count << endl;
                }


                //ProcessOneObject
                ProcessOneObject(itmview, obj_inst_ptr);

                //re-init the bool of newObject
                newObject = true;
            }
        }
        //Insert function: prepare tracking with all objs
        t_controller->Prepare(t_state.get(), BG_object_ptr->getRenderState().get(), this->obj_inst_ptr_vector,
                              visualisationEngine_BG);
//  prepareTrackingWithAllObj();
    }

//not used anymore
    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::prepareTrackingWithAllObj() {
        //make this vectors in the so no need
        std::vector<ITMLib::ITMView *> view_ptr_vec;
        view_ptr_vec.reserve(number_activeObjects);

        std::vector<ITMLib::ITMScene<TVoxel, TIndex> *> scene_ptr_vec;
        scene_ptr_vec.reserve(number_totalObjects);

        for (size_t i = 0; i < obj_inst_ptr_vector.size(); ++i) {
            std::shared_ptr<ObjectInstance<TVoxel, TIndex>>
                    obj_inst_ptr = obj_inst_ptr_vector.at(i);
            if (obj_inst_ptr->checkIsBackground()) {
                view_ptr_vec.insert(view_ptr_vec.begin(), obj_inst_ptr->getCurrentView().get());
                scene_ptr_vec.insert(scene_ptr_vec.begin(), obj_inst_ptr->getScene().get());
            } else {
                view_ptr_vec.push_back(obj_inst_ptr->getCurrentView().get());
                scene_ptr_vec.push_back(obj_inst_ptr->getScene().get());
            }

        }

        t_controller->Prepare(t_state.get(),
                              BG_object_ptr->getRenderState().get(),
                              scene_ptr_vec,
                              view_ptr_vec,
                              visualisationEngine_BG);
    }

    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::ProcessOneObject(std::shared_ptr<ITMLib::ITMView> &itmview,
                                                                ObjectInstance_ptr <TVoxel, TIndex> obj_inst_ptr) {


        auto scene = obj_inst_ptr->getScene().get();

        if (!obj_inst_ptr.get()->checkIsBackground()) {
            sceneIsBackground = false;
//            cout<<"sceneIsBackground "<<sceneIsBackground<<endl;
            std::shared_ptr<ITMLib::ITMTrackingState> tmp_t_state = obj_inst_ptr->getTrackingState();

            tmp_t_state->Reset();

            //TODO dataset living0n hat segfault here. Pose d has no info at Frame 27
            tmp_t_state->pose_d->SetFrom(this->t_state->pose_d);
            tmp_t_state->trackerResult = ITMLib::ITMTrackingState::TRACKING_GOOD;

            denseMapper->ProcessFrame(itmview.get(), tmp_t_state.get(), scene, obj_inst_ptr->getRenderState().get(),
                                      true);

            denseMapper->UpdateVisibleList(itmview.get(), tmp_t_state.get(), scene,
                                           obj_inst_ptr->getRenderState().get(), true);

            t_controller->Prepare(tmp_t_state.get(),
                                  scene,
                                  obj_inst_ptr.get()->getAnchorView_ITM(),
                                  visualisationEngine,
                                  obj_inst_ptr->getRenderState().get());

        } else {
            sceneIsBackground = true;
//            cout<<"sceneIsBackground "<<sceneIsBackground<<endl;
            this->t_state->trackerResult = ITMLib::ITMTrackingState::TRACKING_GOOD;
            denseMapper->ProcessFrame(obj_inst_ptr->getCurrentView().get(),
                                      this->t_state.get(),
                                      scene,
                                      obj_inst_ptr->getRenderState().get(),
                                      true);

            denseMapper->UpdateVisibleList(obj_inst_ptr->getCurrentView().get(),
                                           this->t_state.get(),
                                           scene,
                                           obj_inst_ptr->getRenderState().get(),
                                           true);

        }

    }


    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::outputAllObjImages() {

        cout << "Number of Objects = " << number_totalObjects << endl;
        BG_VoxelCleanUp();

        Matrix3f R(1,0,0,0,0,-1,0,1,0);
        Vector3f T(0,1.5,8);
        auto * birdView = new ORUtils::SE3Pose(R,T);

#ifdef WITH_OPENMP
#pragma omp parallel for private(sceneIsBackground)
#endif
        for (size_t i = 0; i < this->label_ptr_vector.size(); ++i) {
            std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>>
                    label_ptr = label_ptr_vector.at(i);
            std::vector<ObjectInstance_ptr<TVoxel, TIndex>>
                    &
                    obj_inst_vec = (label_ptr.get()->getObjPtrVector());
            cout << *label_ptr.get() << " : " << obj_inst_vec.size() << endl;
#ifdef WITH_OPENMP
#pragma omp parallel for private(sceneIsBackground)
#endif
            for (size_t j = 0; j < obj_inst_vec.size(); ++j) {
                ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr = obj_inst_vec.at(j);

                auto scene = obj_inst_ptr->getScene();

                auto img = std::make_shared<ObjUChar4Image>(imgSize, MEMORYDEVICE_CPU);

                if (obj_inst_ptr->getLabelIndex() != 0) {
                    sceneIsBackground = false;

                    visualisationEngine->FindVisibleBlocks(scene.get(),birdView,
//                                                           this->t_state->pose_d,
                                                           &obj_inst_ptr->getCurrentView()->calib.intrinsics_d,
                                                           obj_inst_ptr->getRenderState().get());

                    visualisationEngine->CreateExpectedDepths(scene.get(), birdView,
//                                                              this->t_state->pose_d,
                                                              &obj_inst_ptr->getCurrentView()->calib.intrinsics_d,
                                                              obj_inst_ptr->getRenderState().get());

                    if (!((ITMLib::ITMRenderState_VH *) obj_inst_ptr->getRenderState().get())->noVisibleEntries >
                        0){
                        Object_Cleanup(obj_inst_ptr);
                        auto scene = obj_inst_ptr.get()->getScene();
                        string stlname = obj_inst_ptr->getClassLabel()->getLabelClassName() + "." + to_string(j) + "_cleaned.stl";
                        SaveSceneToMesh(stlname.c_str(), scene);
                        continue;
                    }





                    visualisationEngine->RenderImage(scene.get(),birdView,
//                                                     this->t_state->pose_d,
                                                     &obj_inst_ptr.get()->getAnchorView_ITM()->calib.intrinsics_d,
                                                     obj_inst_ptr->getRenderState().get(),
                                                     obj_inst_ptr->getRenderState()->raycastImage,
                                                     ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_COLOUR_FROM_VOLUME,
                                                     ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_FROM_NEW_RAYCAST);

                    visualisationEngine->RenderImage(scene.get(),birdView,
//                                                     this->t_state->pose_d,
                                                     &obj_inst_ptr.get()->getAnchorView_ITM()->calib.intrinsics_d,
                                                     obj_inst_ptr->getRenderState().get(),
                                                     BG_object_ptr->getRenderState()->raycastImage,
                                                     ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_COLOUR_FROM_VOLUME,
                                                     ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_FROM_NEW_RAYCAST);

                    img->ChangeDims(obj_inst_ptr->getRenderState().get()->raycastImage->noDims);
                    img->SetFrom(obj_inst_ptr->getRenderState().get()->raycastImage,
                                 ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);

                } else {
                    sceneIsBackground = true;

                    visualisationEngine_BG->FindVisibleBlocks(scene.get(),birdView,
//                                                           this->t_state->pose_d,
                                                           &obj_inst_ptr->getCurrentView()->calib.intrinsics_d,
                                                           obj_inst_ptr->getRenderState().get());

                    visualisationEngine_BG->CreateExpectedDepths(scene.get(),birdView,
//                                                              this->t_state->pose_d,
                                                              &obj_inst_ptr->getCurrentView()->calib.intrinsics_d,
                                                              obj_inst_ptr->getRenderState().get());

                    visualisationEngine_BG->RenderImage(scene.get(), birdView,
//                                                        this->t_state->pose_d,
                                                        &obj_inst_ptr.get()->getAnchorView_ITM()->calib.intrinsics_d,
                                                        obj_inst_ptr->getRenderState().get(),
                                                        obj_inst_ptr->getRenderState()->raycastImage,
                                                        ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_COLOUR_FROM_VOLUME,
                                                        ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_FROM_NEW_RAYCAST);
                    img->ChangeDims(obj_inst_ptr->getRenderState().get()->raycastImage->noDims);
                    img->SetFrom(obj_inst_ptr->getRenderState().get()->raycastImage,
                                 ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
                }

                string name =
                        "Label_" + label_ptr.get()->getLabelClassName() + ".Object" + to_string(j) + ".Frame" +
                        to_string(imgNumber)
                        + ".ppm";

                SaveImageToFile(img.get(), name.c_str());
            }
        }
        //save stl
        if (saveSTL && imgNumber % STL_Frequency == 0) {
//#pragma omp parallel for private(sceneIsBackground)
            for (size_t i = 0; i < this->obj_inst_ptr_vector.size(); ++i) {
                ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr = obj_inst_ptr_vector.at(i);
                sceneIsBackground = obj_inst_ptr->checkIsBackground();
                auto scene = obj_inst_ptr.get()->getScene();
                string stlname = obj_inst_ptr->getClassLabel()->getLabelClassName() + "." + to_string(i) + ".stl";
                SaveSceneToMesh(stlname.c_str(), scene);
            }
        }
    }

    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::UpdateVisibilityOfAllObj() {
        if (obj_inst_ptr_vector.size() == 0) return;
        sceneIsBackground = false;

#pragma omp parallel for private(sceneIsBackground)
        for (size_t i = 1; i < this->obj_inst_ptr_vector.size(); ++i) { //start from1 to skip BG
            const ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr = obj_inst_ptr_vector.at(i);
            auto const *scene = obj_inst_ptr->getScene().get();
            visualisationEngine->FindVisibleBlocks(scene,
                                                   this->t_state->pose_d,
                                                   &obj_inst_ptr->getCurrentView()->calib.intrinsics_d,
                                                   obj_inst_ptr->getRenderState().get());

            visualisationEngine->CreateExpectedDepths(scene,
                                                   this->t_state->pose_d,
                                                   &obj_inst_ptr->getCurrentView()->calib.intrinsics_d,
                                                   obj_inst_ptr->getRenderState().get());
            obj_inst_ptr->updateVisibility();
        }
    }

//check if same obj by 2d overlap
    template<class TVoxel, class TIndex>
    bool ObjSLAMMappingEngine<TVoxel, TIndex>::checkIsSameObject2D(ObjectInstance_ptr <TVoxel, TIndex> obj_ptr_1,
                                                                   ObjectInstance_ptr <TVoxel, TIndex> obj_ptr_2) {
//  cout<<"checkIsSameObject2D\n";
        bool isSame = false;
        ObjSLAM::ObjFloatImage *first = obj_ptr_1.get()->getAnchorView_ITM()->depth;
        ObjSLAM::ObjFloatImage *second = obj_ptr_2.get()->getAnchorView_ITM()->depth;

        auto *firstUchar4 = projectObjectToImg(obj_ptr_1);

        auto *cam = new ObjSLAMCamera(this->calib, this->imgSize);

        auto *pcl = new ORUtils::Image<Vector4f>(imgSize, MEMORYDEVICE_CPU);//in world coordinate
        //TODO change pose to the pose from obj anchor view. add pose to itm view or let the obj inst save the pose itself.
        cam->projectImg2PointCloud(second, pcl, obj_ptr_2->getAnchorView()->getCameraPose().getSE3Pose());
        //  cout<<*this->t_state->pose_d;
//TODO segfault

        ObjFloatImage *out = new ObjFloatImage(imgSize, MEMORYDEVICE_CPU);
        cam->projectPointCloud2Img(pcl, out, obj_ptr_1->getAnchorView()->getCameraPose().getSE3Pose());

        delete cam;
        delete pcl;
        return checkImageOverlap(firstUchar4, second) || checkImageOverlap(first, out);
//  return checkImageOverlap(first, out);
    }

    template<class TVoxel, class TIndex>
    ORUtils::Image<Vector4u> *ObjSLAMMappingEngine<TVoxel,
            TIndex>::projectObjectToImg(ObjectInstance_ptr <TVoxel, TIndex> obj_inst_ptr) {

        sceneIsBackground = obj_inst_ptr->checkIsBackground();

        const auto scene = obj_inst_ptr->getScene();
        //TODO segfault
        visualisationEngine->RenderImage(scene.get(),
                                         this->t_state->pose_d,
                                         &obj_inst_ptr.get()->getAnchorView_ITM()->calib.intrinsics_d,
                                         obj_inst_ptr->getRenderState().get(),
                                         obj_inst_ptr->getRenderState()->raycastImage,
                                         ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_COLOUR_FROM_VOLUME,
                                         ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_FROM_NEW_RAYCAST);
        return obj_inst_ptr->getRenderState()->raycastImage;
    }

    template<class TVoxel, class TIndex>
    ORUtils::Image<Vector4f> *ObjSLAMMappingEngine<TVoxel,
            TIndex>::projectObjectToFloatImg(ObjectInstance_ptr <TVoxel, TIndex> obj_inst_ptr) {

        sceneIsBackground = obj_inst_ptr->checkIsBackground();

        auto scene = obj_inst_ptr->getScene();

        visualisationEngine->RenderImage(scene.get(),
                                         this->t_state->pose_d,
                                         &obj_inst_ptr.get()->getAnchorView_ITM()->calib.intrinsics_d,
                                         obj_inst_ptr->getRenderState().get(),
                                         obj_inst_ptr->getRenderState()->raycastImage,
                                         ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_COLOUR_FROM_VOLUME,
                                         ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_FROM_NEW_RAYCAST);
        return obj_inst_ptr->getRenderState()->raycastResult;
    }

    template<class TVoxel, class TIndex>
    bool ObjSLAMMappingEngine<TVoxel, TIndex>::checkImageOverlap(ObjSLAM::ObjFloatImage *first,
                                                                 ObjSLAM::ObjFloatImage *second) {
//  cout<<"checkImageOverlap\n";
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
//  cout<<x1_min<<" "<<x2_min<<" "<<x1_max<<" "<<x2_max<<"\n";
//  cout<<y1_min<<" "<<y2_min<<" "<<y1_max<<" "<<y2_max<<"\n";
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

    template<class TVoxel, class TIndex>
    bool ObjSLAMMappingEngine<TVoxel, TIndex>::checkImageOverlap(ORUtils::Image<Vector4u> *first,
                                                                 ObjSLAM::ObjFloatImage *second) {
//  cout<<"checkImageOverlap\n";
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
//  cout<<x1_min<<" "<<x2_min<<" "<<x1_max<<" "<<x2_max<<"\n";
//  cout<<y1_min<<" "<<y2_min<<" "<<y1_max<<" "<<y2_max<<"\n";
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
    bool ObjSLAMMappingEngine<TVoxel, TIndex>::checkIsSameObject(ObjectInstance_ptr <TVoxel, TIndex> obj_ptr_1,
                                                                 ObjectInstance_ptr <TVoxel, TIndex> obj_ptr_2) {

        ObjSLAM::ObjFloatImage *first = obj_ptr_1.get()->getAnchorView_ITM()->depth;
        ObjSLAM::ObjFloatImage *second = obj_ptr_2.get()->getAnchorView_ITM()->depth;

        auto *cam = new ObjSLAMCamera(this->calib, this->imgSize);

        auto *pcl1 = new ORUtils::Image<Vector4f>(imgSize, MEMORYDEVICE_CPU);//in world coordinate
        auto *pcl2 = new ORUtils::Image<Vector4f>(imgSize, MEMORYDEVICE_CPU);//in world coordinate

        ORUtils::Vector6<float> cube1 =
                cam->projectImg2PointCloud(first, pcl1,
                                           obj_ptr_1.get()->getAnchorView().get()->getCameraPose().getSE3Pose());
        ORUtils::Vector6<float> cube2 = cam->projectImg2PointCloud(second, pcl2, *this->t_state->pose_d);

        delete cam;
        delete pcl1;
        delete pcl2;

        return checkBoundingCubeOverlap(cube1, cube2);
    }

    template<class TVoxel, class TIndex>
    ORUtils::Vector4<int> ObjSLAMMappingEngine<TVoxel, TIndex>::getBoundingBox(ObjFloatImage *input) {

    }

//using overlapping volumes. TODO add centroid position to further refine the estiamtion
    template<class TVoxel, class TIndex>
    bool ObjSLAMMappingEngine<TVoxel, TIndex>::checkBoundingCubeOverlap(ORUtils::Vector6<float> first,
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

        double v1 = calculateCubeVolume(first);
        double v2 = calculateCubeVolume(second);
        double v_overlap = calculateCubeVolume(overlap);

//  cout << "check" << min(v1, v2) / max(v1, v2) << " " << v_overlap / min(v1, v2) << endl;

        if (v1 <= v2) {
            return v1 / v2 > threshold_volumeChange && v_overlap / v1 > threshold_overlap;
        } else {
            return v2 / v1 > threshold_volumeChange && v_overlap / v2 > threshold_overlap;
        }
    }

    template<class TVoxel, class TIndex>
    double ObjSLAMMappingEngine<TVoxel, TIndex>::calculateCubeVolume(ORUtils::Vector6<float> corners) {
        double l = (corners[3] - corners[0]);
        double w = (corners[4] - corners[1]);
        double h = (corners[5] - corners[2]);
        return l * w * h;
    }

    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::UpdateTrackingState(const ORUtils::SE3Pose *_pose) {
//  cout << "UpdateTrackingState...\n";
        if (this->t_state == NULL) {
            this->t_state = new ITMLib::ITMTrackingState(imgSize, MEMORYDEVICE_CPU);
        }
        this->t_state->pose_d->SetFrom(_pose);
        this->t_state->pose_d->Coerce();

        this->UpdateViewPose();

//  this->t_state->Reset();
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
        this->view->setCameraPose(_pose);
    }

    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::SetTrackingController(
            shared_ptr<ITMLib::ITMTrackingController> _t_controller) {
        t_controller = _t_controller;
    }

    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::outputAllLabelStats() {
        for (size_t i = 0; i < this->label_ptr_vector.size(); ++i) {
            std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>>
                    label_ptr = label_ptr_vector.at(i);
            std::vector<ObjectInstance_ptr<TVoxel, TIndex>>
                    &
                    obj_inst_vec = (label_ptr.get()->getObjPtrVector());
            cout << "Label " << *label_ptr.get()/*->getLabelClassName()*/<< " : " << obj_inst_vec.size() << endl;
        }
    }

    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::deleteAll() {
        delete this->visualisationEngine;
        delete this->visualisationEngine_BG;
        delete this->denseMapper;
//        delete[] preAllocation_array;
    }

/*    template<class TVoxel, class TIndex>
    void ObjSLAMMappingEngine<TVoxel, TIndex>::preAllocateObjects(int n){
//        sceneIsBackground=true;
//        auto bg_scene = std::make_shared<ObjectInstanceScene<TVoxel,TIndex>>(sceneParams_ptr.get(), false, MEMORYDEVICE_CPU);
        sceneIsBackground=false;

        preAllocation_array = new std::shared_ptr<ObjectInstanceScene<TVoxel,TIndex>>[n];

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
        for(int i=0; i<n;++i){
            auto scene = std::make_shared<ObjectInstanceScene<TVoxel,TIndex>>(sceneParams_ptr.get(), false, MEMORYDEVICE_CPU);
            preAllocation_array[n] = scene;
        }
    }*/

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
    void ObjSLAMMappingEngine<TVoxel, TIndex>::Object_Cleanup(ObjectInstance_ptr <TVoxel, TIndex> object) {
        float threshold = 0.8;

        auto scene = object->getScene();
        short object_view_count = object->view_count;

        //stuffs varry over from ResetScene
        int numBlocks = scene->index.getNumAllocatedVoxelBlocks();
        int blockSize = scene->index.getVoxelBlockSize();
        auto *renderState_vh = (ITMLib::ITMRenderState_VH *) object->getRenderState().get();
        int *visibleEntryIds = renderState_vh->GetVisibleEntryIDs();
        int noVisibleEntries = renderState_vh->noVisibleEntries;

        TVoxel *voxelBlocks_ptr = scene->localVBA.GetVoxelBlocks();
        ITMHashEntry *hashTable = scene->index.GetEntries();
        TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
//        for (int entryId = 0; entryId < noVisibleEntries; entryId++) {
        for (int i = 0; i < (sceneIsBackground? scene->index.noTotalEntries_BG:scene->index.noTotalEntries); ++i)  {

            const ITMHashEntry &currentHashEntry = hashTable[i];

            if (currentHashEntry.ptr < 0) continue;


            TVoxel *localVoxelBlock = &(localVBA[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);

            for (int locId = 0; locId < SDF_BLOCK_SIZE3; locId++) {

                if ((float) (localVoxelBlock[locId].view_count) / (float) (object_view_count) < threshold) {
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
            if (!obj_inst_ptr->checkIsBackground()) getVoxelPosFromScene(voxelPos_vec, obj_inst_ptr);
        }

        int size = voxelPos_vec.size();

        ITMLib::ITMScene<TVoxel, TIndex> *scene_BG = this->BG_object_ptr->getScene().get();
        sceneIsBackground = true;
        for (size_t i = 0; i < size; ++i) {
            Vector3s blockPos = voxelPos_vec.at(i);
            int hashIdx = hashIndex(blockPos);
            TVoxel *voxelData = scene_BG->localVBA.GetVoxelBlocks();
            int *voxelAllocationList = scene_BG->localVBA.GetAllocationList();
            int noAllocatedVoxelEntries = scene_BG->localVBA.lastFreeBlockId;

            //loop in the hashBucket
            while (true) {
//      ITMHashEntry hashEntry = voxelIndex[hashIdx];
                ITMHashEntry hashEntry = scene_BG->index.GetEntries()[hashIdx];

                if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= 0) {
//        cache.blockPos = blockPos; cache.blockPtr = hashEntry.ptr * SDF_BLOCK_SIZE3; //khuang: actually no need to use cache
//        vmIndex = hashIdx + 1; // add 1 to support legacy true / false operations for isFound

                    int blockIdx = hashEntry.ptr * SDF_BLOCK_SIZE3;
                    if (hashEntry.ptr >= 0) {

//          voxelAllocationList[vbaIdx + 1] = hashEntry.ptr;

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
    getVoxelPosFromScene(std::vector<Vector3s> &voxelPos_vec, ObjectInstance_ptr <TVoxel, TIndex> obj_ptr) {

        ITMLib::ITMScene<TVoxel, TIndex> *scene = obj_ptr->getScene().get();
//  TIndex index = scene->index;
//  ITMLib::ITMLocalVBA<TVoxel> locVBA = scene->localVBA;
        ITMLib::ITMRenderState_VH *renderState_vh = (ITMLib::ITMRenderState_VH *) obj_ptr->getRenderState().get();
        int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();
        const typename ITMLib::ITMVoxelBlockHash::IndexData *voxelIndex = scene->index.getIndexData();


        for (int blockNo = 0; blockNo < renderState_vh->noVisibleEntries; ++blockNo) {
            int blockID = visibleEntryIDs[blockNo];
            ITMHashEntry &blockData(scene->index.GetEntries()[blockID]);
            voxelPos_vec.push_back(blockData.pos);
//    cout << "BlockNo" << blockNo << " Pos" << blockData.pos << endl;
        }

    }

}