
#ifndef IMAGESOURCE_H
#define IMAGESOURCE_H
#include "InputSource/BaseImageSourceEngine.h"

#include <string>
using InputSource::BaseImageSourceEngine;

namespace ObjSLAM {


/**
  * class ImageSource
  * 
  */

class ImageSource : public BaseImageSourceEngine
{
public:

	// Constructors/Destructors
	//  


	/**
	 * Empty Constructor
	 */
	ImageSource ();

	/**
	 * Empty Destructor
	 */
	virtual ~ImageSource ();

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

	new_class_9 RGBImage;
	new_class_11 DepthImage;
public:


	// Private attribute accessor methods
	//  

private:

public:


	// Private attribute accessor methods
	//  


	/**
	 * Set the value of RGBImage
	 * @param new_var the new value of RGBImage
	 */
	void setRGBImage (new_class_9 new_var)	 {
			RGBImage = new_var;
	}

	/**
	 * Get the value of RGBImage
	 * @return the value of RGBImage
	 */
	new_class_9 getRGBImage ()	 {
		return RGBImage;
	}

	/**
	 * Set the value of DepthImage
	 * @param new_var the new value of DepthImage
	 */
	void setDepthImage (new_class_11 new_var)	 {
			DepthImage = new_var;
	}

	/**
	 * Get the value of DepthImage
	 * @return the value of DepthImage
	 */
	new_class_11 getDepthImage ()	 {
		return DepthImage;
	}
private:


	void initAttributes () ;

};
} // end of package namespace

#endif // IMAGESOURCE_H
