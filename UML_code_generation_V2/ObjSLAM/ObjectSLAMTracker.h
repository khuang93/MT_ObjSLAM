
#ifndef OBJECTSLAMTRACKER_H
#define OBJECTSLAMTRACKER_H
#include "ITMLib/ITMTracker.h"

#include <string>
using ITMLib::ITMTracker;

namespace ObjSLAM {


/**
  * class ObjectSLAMTracker
  * 
  */

class ObjectSLAMTracker : virtual public ITMTracker
{
public:

	// Constructors/Destructors
	//  


	/**
	 * Empty Constructor
	 */
	ObjectSLAMTracker ();

	/**
	 * Empty Destructor
	 */
	virtual ~ObjectSLAMTracker ();

	// Static Public attributes
	//  

	// Public attributes
	//  


	// Public attribute accessor methods
	//  


	// Public attribute accessor methods
	//  



	/**
	 * @param  trackingState
	 * @param  semanticView
	 */
	void TrackCamera (ITMLib::ITMTrackingState trackingState, ObjSLAM::ObjectView semanticView)
	{
	}


	/**
	 * @param  poseGraph
	 */
	void updatePoseGraph (ObjSLAM::PoseGraph poseGraph)
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

public:


	// Private attribute accessor methods
	//  

private:

public:


	// Private attribute accessor methods
	//  

private:



};
} // end of package namespace

#endif // OBJECTSLAMTRACKER_H
