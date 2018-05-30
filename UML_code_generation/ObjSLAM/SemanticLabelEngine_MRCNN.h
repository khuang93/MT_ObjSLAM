
#ifndef SEMANTICLABELENGINE_MRCNN_H
#define SEMANTICLABELENGINE_MRCNN_H
#include "ObjSLAM/SemanticLabelEngine.h"

#include <string>

namespace ObjSLAM {


/**
  * class SemanticLabelEngine_MRCNN
  * use TensorFlow C++ API
  * transfer the API of
  */

class SemanticLabelEngine_MRCNN : public SemanticLabelEngine
{
public:

	// Constructors/Destructors
	//  


	/**
	 * Empty Constructor
	 */
	SemanticLabelEngine_MRCNN ();

	/**
	 * Empty Destructor
	 */
	virtual ~SemanticLabelEngine_MRCNN ();

	// Static Public attributes
	//  

	// Public attributes
	//  


	// Public attribute accessor methods
	//  


	// Public attribute accessor methods
	//  



	/**
	 * Here should run the mask rcnn implementation
	 * 
	 */
	void MRCNN_Runner ()
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

	// Path of the MRCNN model
	// 
	undef ModelPath;
	bool useGPU;
public:


	// Private attribute accessor methods
	//  

private:

public:


	// Private attribute accessor methods
	//  


	/**
	 * Set the value of ModelPath
	 * Path of the MRCNN model
	 * 
	 * @param new_var the new value of ModelPath
	 */
	void setModelPath (undef new_var)	 {
			ModelPath = new_var;
	}

	/**
	 * Get the value of ModelPath
	 * Path of the MRCNN model
	 * 
	 * @return the value of ModelPath
	 */
	undef getModelPath ()	 {
		return ModelPath;
	}

	/**
	 * Set the value of useGPU
	 * @param new_var the new value of useGPU
	 */
	void setUseGPU (bool new_var)	 {
			useGPU = new_var;
	}

	/**
	 * Get the value of useGPU
	 * @return the value of useGPU
	 */
	bool getUseGPU ()	 {
		return useGPU;
	}
private:


	void initAttributes () ;

};
} // end of package namespace

#endif // SEMANTICLABELENGINE_MRCNN_H
