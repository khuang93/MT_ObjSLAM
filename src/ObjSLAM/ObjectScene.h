
#ifndef OBJECTSCENE_H
#define OBJECTSCENE_H
#include "ITMLib/ITMScene.h"
#include "ObjSLAM/ObjectView.h"

#include <string>
using ITMLib::ITMScene;

namespace ObjSLAM {


/**
  * class ObjectScene
  * 
  */

class ObjectScene : public ITMScene, public ObjectView
{
public:

	// Constructors/Destructors
	//  


	/**
	 * Empty Constructor
	 */
	ObjectScene ();

	/**
	 * Empty Destructor
	 */
	virtual ~ObjectScene ();

	// Static Public attributes
	//  

	// Public attributes
	//  


	// Public attribute accessor methods
	//  


	// Public attribute accessor methods
	//  


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

	std::vector<ObjSLAM::ObjectInstance> listOfAllObjects;
	ObjSLAM::ObjectsGraph objectsGraph;
	std::vector<ObjSLAM::SemanticView> listOfAllViews;
	ObjSLAM::PoseGraph poseGraph;
public:


	// Private attribute accessor methods
	//  

private:

public:


	// Private attribute accessor methods
	//  


	/**
	 * Set the value of listOfAllObjects
	 * @param new_var the new value of listOfAllObjects
	 */
	void setListOfAllObjects (std::vector<ObjSLAM::ObjectInstance> new_var)	 {
			listOfAllObjects = new_var;
	}

	/**
	 * Get the value of listOfAllObjects
	 * @return the value of listOfAllObjects
	 */
	std::vector<ObjSLAM::ObjectInstance> getListOfAllObjects ()	 {
		return listOfAllObjects;
	}

	/**
	 * Set the value of objectsGraph
	 * @param new_var the new value of objectsGraph
	 */
	void setObjectsGraph (ObjSLAM::ObjectsGraph new_var)	 {
			objectsGraph = new_var;
	}

	/**
	 * Get the value of objectsGraph
	 * @return the value of objectsGraph
	 */
	ObjSLAM::ObjectsGraph getObjectsGraph ()	 {
		return objectsGraph;
	}

	/**
	 * Set the value of listOfAllViews
	 * @param new_var the new value of listOfAllViews
	 */
	void setListOfAllViews (std::vector<ObjSLAM::SemanticView> new_var)	 {
			listOfAllViews = new_var;
	}

	/**
	 * Get the value of listOfAllViews
	 * @return the value of listOfAllViews
	 */
	std::vector<ObjSLAM::SemanticView> getListOfAllViews ()	 {
		return listOfAllViews;
	}

	/**
	 * Set the value of poseGraph
	 * @param new_var the new value of poseGraph
	 */
	void setPoseGraph (ObjSLAM::PoseGraph new_var)	 {
			poseGraph = new_var;
	}

	/**
	 * Get the value of poseGraph
	 * @return the value of poseGraph
	 */
	ObjSLAM::PoseGraph getPoseGraph ()	 {
		return poseGraph;
	}
private:


	void initAttributes () ;

};
} // end of package namespace

#endif // OBJECTSCENE_H
