
#ifndef OBJECTSCENERECONSTRUCTIONENGINE_H
#define OBJECTSCENERECONSTRUCTIONENGINE_H
#include "../../External/InfiniTAM/InfiniTAM/ITMLib/Engines/Reconstruction/Interface/ITMSceneReconstructionEngine.h"


#include <string>
#include "SemanticLabelEngine.h"
using ITMLib::ITMSceneReconstructionEngine;
using ITMLib::ITMSceneReconstructionEngine_CUDA_TVoxel_ITMVoxelBlockHash_;


namespace ObjSLAM {


/**
  * class ObjectSceneReconstructionEngine
  * 
  */

class ObjectSceneReconstructionEngine : virtual public ITMSceneReconstructionEngine, public ITMSceneReconstructionEngine_CUDA_TVoxel_ITMVoxelBlockHash_
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
	 * @param  objectView
	 * @param  objectScene
	 */
	void LabelAllocatedVoxels (ObjSLAM::ObjectView objectView, ObjSLAM::ObjectScene objectScene)
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


	/**
	 */
	void initSceneReconstructionEngine ()
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
	ITMLib::ITMSceneReconstructionEngine_CPU< TVoxel, ITMVoxelBlockHash > cpuEngine;
	ITMLib::ITMSceneReconstructionEngine_CUDA< TVoxel, ITMVoxelBlockHash > cudaEngine;
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

	/**
	 * Set the value of cpuEngine
	 * @param new_var the new value of cpuEngine
	 */
	void setCpuEngine (ITMLib::ITMSceneReconstructionEngine_CPU< TVoxel, ITMVoxelBlockHash > new_var)	 {
			cpuEngine = new_var;
	}

	/**
	 * Get the value of cpuEngine
	 * @return the value of cpuEngine
	 */
	ITMLib::ITMSceneReconstructionEngine_CPU< TVoxel, ITMVoxelBlockHash > getCpuEngine ()	 {
		return cpuEngine;
	}

	/**
	 * Set the value of cudaEngine
	 * @param new_var the new value of cudaEngine
	 */
	void setCudaEngine (ITMLib::ITMSceneReconstructionEngine_CUDA< TVoxel, ITMVoxelBlockHash > new_var)	 {
			cudaEngine = new_var;
	}

	/**
	 * Get the value of cudaEngine
	 * @return the value of cudaEngine
	 */
	ITMLib::ITMSceneReconstructionEngine_CUDA< TVoxel, ITMVoxelBlockHash > getCudaEngine ()	 {
		return cudaEngine;
	}
private:


	void initAttributes () ;

};
} // end of package namespace

#endif // OBJECTSCENERECONSTRUCTIONENGINE_H
