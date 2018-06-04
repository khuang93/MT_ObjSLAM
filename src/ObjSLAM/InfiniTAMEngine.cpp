#include "InfiniTAMEngine.h"

#include "InfiniTAMApp.h"
#include "ITMLib/ITMBasicEngine.h"

#include <string>
using ITMLib::ITMMainEngine;

namespace ObjSLAM {


/**
  * class InfiniTAMEngine
  * Communicate with ITM
  * 
  */

class InfiniTAMEngine : public InfiniTAMApp, public ITMMainEngine
{
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
 * @return ITMView*
 */
ITMView* getCurrentView ()
{
}


/**
 * @return ITMLib::ITMScene
 */
ITMLib::ITMScene getScene ()
{
}


/**
 * @return ITMLib::ITMVoxelBlockHash
 */
ITMLib::ITMVoxelBlockHash getVoxelBlock ()
{
}

protected:

// Static Protected attributes
//  

// Protected attributes
//  

public:


// Protected attribute accessor methods
//  

protected:

public:


// Protected attribute accessor methods
//  

protected:


private:

// Static Private attributes
//  

// Private attributes
//  

ObjSLAM::ObjectSLAMTracker tracker;
ObjSLAM::ObjectSceneReconstructionEngine objectSceneReconstructionEngine;
InputSource::ImageFileReader inputSource;
public:


// Private attribute accessor methods
//  

private:

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

/**
 * Set the value of objectSceneReconstructionEngine
 * @param new_var the new value of objectSceneReconstructionEngine
 */
void setObjectSceneReconstructionEngine (ObjSLAM::ObjectSceneReconstructionEngine new_var) {
	objectSceneReconstructionEngine = new_var;
}

/**
 * Get the value of objectSceneReconstructionEngine
 * @return the value of objectSceneReconstructionEngine
 */
ObjSLAM::ObjectSceneReconstructionEngine getObjectSceneReconstructionEngine () {
	return objectSceneReconstructionEngine;
}

/**
 * Set the value of inputSource
 * @param new_var the new value of inputSource
 */
void setInputSource (InputSource::ImageFileReader new_var) {
	inputSource = new_var;
}

/**
 * Get the value of inputSource
 * @return the value of inputSource
 */
InputSource::ImageFileReader getInputSource () {
	return inputSource;
}
private:


void initAttributes () ;

};
} // end of package namespace
// Constructors/Destructors
//  

InfiniTAMEngine::InfiniTAMEngine () {
initAttributes();
}

InfiniTAMEngine::~InfiniTAMEngine () { }

//  
// Methods
//  


// Accessor methods
//  


// Other methods
//  

void InfiniTAMEngine::initAttributes () {
}

