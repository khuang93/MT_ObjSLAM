#include "InfiniTAMCreator.h"

#include "InfiniTAMApp.h"
#include "ITMLib/ITMBasicEngine.h"

#include <string>
using ITMLib::ITMBasicEngine;

namespace ObjSLAM {


/**
  * class InfiniTAMCreator
  * Communicate with ITM
  * 
  */

class InfiniTAMCreator : public InfiniTAMApp, public ITMBasicEngine
{
public:

// Constructors/Destructors
//  


/**
 * Empty Constructor
 */
InfiniTAMCreator ();

/**
 * Empty Destructor
 */
virtual ~InfiniTAMCreator ();

// Static Public attributes
//  

// Public attributes
//  


// Public attribute accessor methods
//  


// Public attribute accessor methods
//  



/**
 */
void getCurrentView ()
{
}


/**
 */
void getScene ()
{
}


/**
 */
void getVoxelBlock ()
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
private:


void initAttributes () ;

};
} // end of package namespace
// Constructors/Destructors
//  

InfiniTAMCreator::InfiniTAMCreator () {
initAttributes();
}

InfiniTAMCreator::~InfiniTAMCreator () { }

//  
// Methods
//  


// Accessor methods
//  


// Other methods
//  

void InfiniTAMCreator::initAttributes () {
}

