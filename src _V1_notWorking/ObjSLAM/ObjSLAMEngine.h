
#ifndef OBJSLAMENGINE_H
#define OBJSLAMENGINE_H

#include <string>
#include <iostream>

#include "InfiniTAMEngine.h"
#include "ObjectScene.h"
#include "LoopClosureDetector.h"
#include "ObjectSLAMTracker.h"
#include "ObjectSceneReconstructionEngine.h"

namespace ObjSLAM {
//template<class TVoxel, class TIndex>

/**
  * class ObjSLAMEngine
  * 
  */

class ObjSLAMEngine
{
public:

	// Constructors/Destructors
	//  


	/**
	 * Empty Constructor
	 */
	ObjSLAMEngine ();

	/**
	 * Empty Destructor
	 */
	virtual ~ObjSLAMEngine ();

	// Static Public attributes
	//  

	// Public attributes
	//  


	// Public attribute accessor methods
	//  


	// Public attribute accessor methods
	//  



	/**
	 * @return ObjSLAM::InfiniTAMEngine
	 */
	ObjSLAM::InfiniTAMEngine init_InfiniTAMCreator ()
	{
	}


	/**
	 */
	void init_SemanticLabelGreator ()
	{
	}


	/**
	 * @return ObjSLAM::ObjectSceneReconstructionEngine
	 */
	ObjSLAM::ObjectSceneReconstructionEngine init_ObjectSceneReconstructionEngine ()
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

	ObjSLAM::InfiniTAMEngine<TVoxel,TIndex> infiniTAMCreator;
	ObjSLAM::LoopClosureDetector loopClosureDetector;
	ObjSLAM::ObjectScene scene;
	ObjSLAM::ObjectSLAMTracker objSLAMTracker;
public:


	// Private attribute accessor methods
	//

private:

public:


	// Private attribute accessor methods
	//


	/**
	 * Set the value of infiniTAMCreator
	 * @param new_var the new value of infiniTAMCreator
	 */
//	void setInfiniTAMCreator (ObjSLAM::InfiniTAMEngine new_var)	 {
//			infiniTAMCreator = new_var;
//	}
//
//	/**
//	 * Get the value of infiniTAMCreator
//	 * @return the value of infiniTAMCreator
//	 */
//	ObjSLAM::InfiniTAMEngine getInfiniTAMCreator ()	 {
//		return infiniTAMCreator;
//	}

	/**
	 * Set the value of loopClosureDetector
	 * @param new_var the new value of loopClosureDetector
	 */
	void setLoopClosureDetector (ObjSLAM::LoopClosureDetector new_var)	 {
			loopClosureDetector = new_var;
	}

	/**
	 * Get the value of loopClosureDetector
	 * @return the value of loopClosureDetector
	 */
	ObjSLAM::LoopClosureDetector getLoopClosureDetector ()	 {
		return loopClosureDetector;
	}

	/**
	 * Set the value of scene
	 * @param new_var the new value of scene
	 */
	void setScene (ObjSLAM::ObjectScene new_var)	 {
			scene = new_var;
	}

	/**
	 * Get the value of scene
	 * @return the value of scene
	 */
	ObjSLAM::ObjectScene getScene ()	 {
		return scene;
	}

	/**
	 * Set the value of objSLAMTracker
	 * @param new_var the new value of objSLAMTracker
	 */
	void setObjSLAMTracker (ObjSLAM::ObjectSLAMTracker new_var)	 {
			objSLAMTracker = new_var;
	}

	/**
	 * Get the value of objSLAMTracker
	 * @return the value of objSLAMTracker
	 */
	ObjSLAM::ObjectSLAMTracker getObjSLAMTracker ()	 {
		return objSLAMTracker;
	}
private:


	void initAttributes () ;

};
} // end of package namespace

#endif // OBJSLAMENGINE_H
