//
// Created by khuang on 10/9/18.
//

#ifndef MT_OBJSLAM_OBJSLAMFUNCTIONS_H
#define MT_OBJSLAM_OBJSLAMFUNCTIONS_H

#include "ObjSLAMDataTypes.h"

namespace ObjSLAM{
    /*<template class TVoxel, template class TIndex>
    void MultiRaycast(std::vector<ObjectInstance_ptr<TVoxel,TIndex>> obj_inst_ptr_vec, Matrix4f invM){
        Matrix4f M;
        invM.inv(M);


#ifdef WITH_OPENMP
#pragma omp parallel for private(sceneIsBackground)
#endif
        for (int locId = 0; locId < imgSize.x * imgSize.y; ++locId) {
            int y = locId / imgSize.x;
            int x = locId - y * imgSize.x;
            int locId2 = (int) floor((float) x / minmaximg_subsample) +
                         (int) floor((float) y / minmaximg_subsample) * imgSize.x;

            double dist = DBL_MAX;
*//*#ifdef WITH_OPENMP
#pragma omp parallel for private(sceneIsBackground)
#endif*//*
            for (size_t i = 0; i < obj_inst_ptr_vector.size(); ++i) {
                sceneIsBackground = i == 0 ? true : false;
                ObjSLAM::ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr = obj_inst_ptr_vector.at(i);
                Vector4f *pt_ray_tmp = obj_inst_ptr->GetRenderState()->raycastResult->GetData(MEMORYDEVICE_CPU);

                Vector4f pt_tmp = pt_ray_tmp[locId];
                if (pt_tmp.w == 0) continue;

                Vector4f pt_tmp_tmp = pt_tmp;

                pt_tmp_tmp.w = 1.0f;

                Vector4f pt_cam_f = M * pt_tmp_tmp;
                double dist_new = NORM3(pt_cam_f);
                if (dist_new < dist) {
                    dist = dist_new;
                    pointsRay_final[locId]=pt_tmp;
                }
            }
        }

    }*/









}


#endif //MT_OBJSLAM_OBJSLAMFUNCTIONS_H
