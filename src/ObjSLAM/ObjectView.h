
#ifndef OBJECTVIEW_H
#define OBJECTVIEW_H
#include "../../External/InfiniTAM/InfiniTAM/ITMLib/Objects/Views/ITMView.h"
#include "Pose.h"

#include <string>
#include <vector>
using ITMLib::ITMView;


namespace ObjSLAM {


/**
  * class ObjectView
  * 
  */

class ObjectView : public ITMView
{
public:

	// Constructors/Destructors
	//  


	/**
	 * Empty Constructor
	 */
	ObjectView ();

	/**
	 * Empty Destructor
	 */
	virtual ~ObjectView ();

	// Static Public attributes
	//  

	// Public attributes
	//  

	//TODO
  // undef segmentationImage;

	// Public attribute accessor methods
	//  


	// Public attribute accessor methods
	//  


	/**
	 * Set the value of segmentationImage
	 * @param new_var the new value of segmentationImage
	 */
//	void setSegmentationImage (undef new_var)	 {
//			segmentationImage = new_var;
//	}

	/**
	 * Get the value of segmentationImage
	 * @return the value of segmentationImage
	 */
//	undef getSegmentationImage ()	 {
//		return segmentationImage;
//	}


	/**
	 * @return undef
	 */
//	undef getSegmentationMask ()
//	{
//	}


	/**
	 */
	void getCamPose ()
	{
	}


	/**
	 * @param  pose
	 */
	void setCamPose (ObjSLAM::Pose pose)
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

	// Type of the list to be discussed
	// 
	std::vector< int >* listOfObjectsInView;
	std::vector<Vector3< T >> distVectorToObject;
	ObjSLAM::Pose camPose;
public:


	// Private attribute accessor methods
	//  

private:

public:


	// Private attribute accessor methods
	//  


	/**
	 * Set the value of listOfObjectsInView
	 * Type of the list to be discussed
	 * 
	 * @param new_var the new value of listOfObjectsInView
	 */
	void setListOfObjectsInView (std::vector< int >* new_var)	 {
			listOfObjectsInView = new_var;
	}

	/**
	 * Get the value of listOfObjectsInView
	 * Type of the list to be discussed
	 * 
	 * @return the value of listOfObjectsInView
	 */
	std::vector< int >* getListOfObjectsInView ()	 {
		return listOfObjectsInView;
	}

	/**
	 * Set the value of distVectorToObject
	 * @param new_var the new value of distVectorToObject
	 */
	void setDistVectorToObject (std::vector<Vector3< T >> new_var)	 {
			distVectorToObject = new_var;
	}

	/**
	 * Get the value of distVectorToObject
	 * @return the value of distVectorToObject
	 */
	std::vector<Vector3< T >> getDistVectorToObject ()	 {
		return distVectorToObject;
	}

	/**
	 * Set the value of camPose
	 * @param new_var the new value of camPose
	 */
	void setCamPose (ObjSLAM::Pose new_var)	 {
			camPose = new_var;
	}

	/**
	 * Get the value of camPose
	 * @return the value of camPose
	 */
	ObjSLAM::Pose getCamPose ()	 {
		return camPose;
	}
private:


	void initAttributes () ;

};
} // end of package namespace

#endif // OBJECTVIEW_H
