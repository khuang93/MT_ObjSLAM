#include "SemanticLabelEngine.h"


#include <string>

namespace ObjSLAM {


/**
  * class SemanticLabelEngine
  * 
  */

class SemanticLabelEngine
{
public:

// Constructors/Destructors
//  


/**
 * Empty Constructor
 */
SemanticLabelEngine ();

/**
 * Empty Destructor
 */
virtual ~SemanticLabelEngine ();

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
virtual void InferSegmentation (ObjSLAM::ObjectView semanticView)
{
}


/**
 */
void GetImage ()
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

// specify the source of the Semantic Labels using a string:
// 
// "NYUDataSet_Groundtruth"
// "TUMDataSet_Groundtruth"
// "MRCNN"
// 
// possible to extend to other methods to provide the semantic sgmentation
// 
undef sourceOfLabel;
ITMUChar4Image* rgbImage;
undef labelMask;
ObjSLAM::ObjectView semanticView;
public:


// Private attribute accessor methods
//  

private:

public:


// Private attribute accessor methods
//  


/**
 * Set the value of sourceOfLabel
 * specify the source of the Semantic Labels using a string:
 * 
 * "NYUDataSet_Groundtruth"
 * "TUMDataSet_Groundtruth"
 * "MRCNN"
 * 
 * possible to extend to other methods to provide the semantic sgmentation
 * 
 * @param new_var the new value of sourceOfLabel
 */
void setSourceOfLabel (undef new_var) {
	sourceOfLabel = new_var;
}

/**
 * Get the value of sourceOfLabel
 * specify the source of the Semantic Labels using a string:
 * 
 * "NYUDataSet_Groundtruth"
 * "TUMDataSet_Groundtruth"
 * "MRCNN"
 * 
 * possible to extend to other methods to provide the semantic sgmentation
 * 
 * @return the value of sourceOfLabel
 */
undef getSourceOfLabel () {
	return sourceOfLabel;
}

/**
 * Set the value of rgbImage
 * @param new_var the new value of rgbImage
 */
void setRgbImage (ITMUChar4Image* new_var) {
	rgbImage = new_var;
}

/**
 * Get the value of rgbImage
 * @return the value of rgbImage
 */
ITMUChar4Image* getRgbImage () {
	return rgbImage;
}

/**
 * Set the value of labelMask
 * @param new_var the new value of labelMask
 */
void setLabelMask (undef new_var) {
	labelMask = new_var;
}

/**
 * Get the value of labelMask
 * @return the value of labelMask
 */
undef getLabelMask () {
	return labelMask;
}

/**
 * Set the value of semanticView
 * @param new_var the new value of semanticView
 */
void setSemanticView (ObjSLAM::ObjectView new_var) {
	semanticView = new_var;
}

/**
 * Get the value of semanticView
 * @return the value of semanticView
 */
ObjSLAM::ObjectView getSemanticView () {
	return semanticView;
}
private:


void initAttributes () ;

};
} // end of package namespace
// Constructors/Destructors
//  

SemanticLabelEngine::SemanticLabelEngine () {
initAttributes();
}

SemanticLabelEngine::~SemanticLabelEngine () { }

//  
// Methods
//  


// Accessor methods
//  


// Other methods
//  

void SemanticLabelEngine::initAttributes () {
}

