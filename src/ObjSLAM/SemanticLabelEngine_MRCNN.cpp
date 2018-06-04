#include "SemanticLabelEngine_MRCNN.h"

// Constructors/Destructors
//  

SemanticLabelEngine_MRCNN::SemanticLabelEngine_MRCNN () {
initAttributes();
}

SemanticLabelEngine_MRCNN::~SemanticLabelEngine_MRCNN () { }

//  
// Methods
//  


// Accessor methods
//  


// Other methods
//  

void SemanticLabelEngine_MRCNN::initAttributes () {
	useGPU = false;
}

