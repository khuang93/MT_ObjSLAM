
#ifndef OBJECTSCENERECONSTRUCTIONENGINE_H
#define OBJECTSCENERECONSTRUCTIONENGINE_H
#include "ITMLib/ITMSceneReconstructionEngine.h"

#include <string>
using ITMLib::ITMSceneReconstructionEngine;

namespace ObjSLAM {


/**
  * class ObjectSceneReconstructionEngine
  * 
  */

class ObjectSceneReconstructionEngine : virtual public ITMSceneReconstructionEngine
{
public:

	// Constructors/Destructors
	//  


	/**
	 * Empty Constructor
	 */
	ObjectSceneReconstructionEngine ();

	/**
	 * Empty Destructor
	 */
	virtual ~ObjectSceneReconstructionEngine ();

	// Static Public attributes
	//  

	// Public attributes
	//  


	// Public attribute accessor methods
	//  


	// Public attribute accessor methods
	//  



	/**
	 * @param  semanticView
	 * @param  semanticScene
	 */
	void LabelAllocatedVoxels (ObjSLAM::SemanticView semanticView, ObjSLAM::SemanticScene semanticScene)
	{
	}


	/**
	 * The MRCNN segmentation in 2D img is not precise enough
	 * 
	 * Refine the 3D Segmentation based on connectivity and edges (compare to the paper
	 * of mask fusion).
	 */
	void RefineVoxelSegmentation ()
	{
	}


	/**
	 * @return ObjSLAM::SemanticLabelEngine
	 */
	ObjSLAM::SemanticLabelEngine CreateSementicLabelEngine ()
	{
	}


	/**
	 */
	void UpdateObjectInstance ()
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

	ObjSLAM::SemanticLabelEngine semanticLabelEngine;
	std::vector<ObjSLAM::ObjectInstance> list_ObjectInstance;
public:


	// Private attribute accessor methods
	//  

private:

public:


	// Private attribute accessor methods
	//  


	/**
	 * Set the value of semanticLabelEngine
	 * @param new_var the new value of semanticLabelEngine
	 */
	void setSemanticLabelEngine (ObjSLAM::SemanticLabelEngine new_var)	 {
			semanticLabelEngine = new_var;
	}

	/**
	 * Get the value of semanticLabelEngine
	 * @return the value of semanticLabelEngine
	 */
	ObjSLAM::SemanticLabelEngine getSemanticLabelEngine ()	 {
		return semanticLabelEngine;
	}

	/**
	 * Set the value of list_ObjectInstance
	 * @param new_var the new value of list_ObjectInstance
	 */
	void setList_ObjectInstance (std::vector<ObjSLAM::ObjectInstance> new_var)	 {
			list_ObjectInstance = new_var;
	}

	/**
	 * Get the value of list_ObjectInstance
	 * @return the value of list_ObjectInstance
	 */
	std::vector<ObjSLAM::ObjectInstance> getList_ObjectInstance ()	 {
		return list_ObjectInstance;
	}
private:


	void initAttributes () ;

};
} // end of package namespace

#endif // OBJECTSCENERECONSTRUCTIONENGINE_H
