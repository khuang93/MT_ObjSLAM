#include "SemanticLabelEngine_Groundtruth.h"

#include "ObjSLAM/SemanticLabelEngine.h"

#include <string>

namespace ObjSLAM {


/**
  * class SemanticLabelEngine_Groundtruth
  * 
  */

class SemanticLabelEngine_Groundtruth : public SemanticLabelEngine
{
public:

// Constructors/Destructors
//  


/**
 * Empty Constructor
 */
SemanticLabelEngine_Groundtruth ();

/**
 * Empty Destructor
 */
virtual ~SemanticLabelEngine_Groundtruth ();

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
 */
void InferSegmentation_NYU (ObjSLAM::SemanticView semanticView)
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

undef nameDataset;
public:


// Private attribute accessor methods
//  

private:

public:


// Private attribute accessor methods
//  


/**
 * Set the value of nameDataset
 * @param new_var the new value of nameDataset
 */
void setNameDataset (undef new_var) {
	nameDataset = new_var;
}

/**
 * Get the value of nameDataset
 * @return the value of nameDataset
 */
undef getNameDataset () {
	return nameDataset;
}
private:


void initAttributes () ;

};
} // end of package namespace
// Constructors/Destructors
//  

SemanticLabelEngine_Groundtruth::SemanticLabelEngine_Groundtruth () {
initAttributes();
}

SemanticLabelEngine_Groundtruth::~SemanticLabelEngine_Groundtruth () { }

//  
// Methods
//  


// Accessor methods
//  


// Other methods
//  

void SemanticLabelEngine_Groundtruth::initAttributes () {
}

