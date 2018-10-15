//
// Created by Kailin on 2018-10-09.
//

#ifndef MT_OBJSLAM_OBJSLAMMULTITRACKINGENGINE_H
#define MT_OBJSLAM_OBJSLAMMULTITRACKINGENGINE_H

#include "ObjSLAMTrackingEngine.h"
#include "ObjSLAMVoxelSceneParams.h"
#include "ObjSLAMDataTypes.h"

namespace ObjSLAM {
    using namespace ITMLib;

    class ObjSLAMMultiTrackingEngine : public ObjSLAMTrackingEngine {

    public:
        ObjSLAMMultiTrackingEngine(const std::shared_ptr<ITMLib::ITMLibSettings> _settings,
                                   const std::shared_ptr<ITMLib::ITMLibSettings> _settings_obj,
                              const std::shared_ptr<ITMLib::ITMRGBDCalib> _calib,
                              const Vector2i _imgSize) : ObjSLAMTrackingEngine(_settings, _settings_obj, _calib, _imgSize) {

        }




        shared_ptr<ITMLib::ITMTrackingState> TrackObjViewFrame(ITMLib::ITMView *view);

        template<class TVoxel, class TIndex>
        shared_ptr<ITMLib::ITMTrackingState>
        TrackFrame(ITMLib::ITMView *view, std::vector<ObjectInstance_ptr<TVoxel, TIndex>> obj_inst_ptr_vec);

    };

}
#endif //MT_OBJSLAM_OBJSLAMMULTITRACKINGENGINE_H
