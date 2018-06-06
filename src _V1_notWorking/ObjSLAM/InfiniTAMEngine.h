//
// Created by khuang on 6/4/18.
//

#ifndef MT_OBJSLAM_INFINITAMENGINE_H
#define MT_OBJSLAM_INFINITAMENGINE_H

#include "../../External/InfiniTAM/InfiniTAM/ITMLib/Core/ITMMainEngine.h"
#include "ObjectSLAMTracker.h"

#include <string>
using ITMLib::ITMMainEngine;

namespace ObjSLAM {
template<class TVoxel, class TIndex>
class InfiniTAMEngine : public ITMLib::ITMMainEngine
{

/**
  * class InfiniTAMEngine
  * Communicate with ITM
  *
  */


 public:

// Constructors/Destructors
//


/**
 * Empty Constructor
 */
  InfiniTAMEngine ();

/**
 * Empty Destructor
 */
  virtual ~InfiniTAMEngine ();

// Static Public attributes
//

// Public attributes
//


// Public attribute accessor methods
//


// Public attribute accessor methods
//



/**
 * @return ITMLib::ITMView*
 */
  ITMLib::ITMView* getCurrentView ();


/**
 * @return ITMLib::ITMScene
 */
//  template<class TVoxel, class TIndex>
//  ITMLib::ITMScene<TVoxel, TIndex> getScene ();


/**
 * @return ITMLib::ITMVoxelBlockHash
 */
//  ITMLib::ITMVoxelBlockHash getVoxelBlock ();

 private:

// Static Private attributes
//

// Private attributes
//

  ObjSLAM::ObjectSLAMTracker tracker;
  ObjSLAM::ObjectSceneReconstructionEngine objectSceneReconstructionEngine;
//  InputSource::ImageFileReader inputSource;

 public:


// Private attribute accessor methods
//


/**
 * Set the value of tracker
 * @param new_var the new value of tracker
 */
  void setTracker (ObjSLAM::ObjectSLAMTracker new_var) {
    tracker = new_var;
  }

/**
 * Get the value of tracker
 * @return the value of tracker
 */
  ObjSLAM::ObjectSLAMTracker getTracker () {
    return tracker;
  }

///**
// * Set the value of objectSceneReconstructionEngine
// * @param new_var the new value of objectSceneReconstructionEngine
// */
//  void setObjectSceneReconstructionEngine (ObjSLAM::ObjectSceneReconstructionEngine new_var) {
//    objectSceneReconstructionEngine = new_var;
//  }
//
///**
// * Get the value of objectSceneReconstructionEngine
// * @return the value of objectSceneReconstructionEngine
// */
//  ObjSLAM::ObjectSceneReconstructionEngine getObjectSceneReconstructionEngine () {
//    return objectSceneReconstructionEngine;
//  }
//
///**
// * Set the value of inputSource
// * @param new_var the new value of inputSource
// */
//  void setInputSource (InputSource::ImageFileReader new_var) {
//    inputSource = new_var;
//  }
//
///**
// * Get the value of inputSource
// * @return the value of inputSource
// */
//  InputSource::ImageFileReader getInputSource () {
//    return inputSource;
//  }
 private:


  void initAttributes ();

};
} // end of package namespace



#endif //MT_OBJSLAM_INFINITAMENGINE_H