
#ifndef OBJECTCLASSLABEL_H
#define OBJECTCLASSLABEL_H

#include <string>

namespace ObjSLAM {


/**
  * class ObjectClassLabel
  * 
  */

class ObjectClassLabel
{
public:

	// Constructors/Destructors
	//  


	/**
	 * Empty Constructor
	 */
	ObjectClassLabel ();

	/**
	 * Empty Destructor
	 */
	virtual ~ObjectClassLabel ();

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

	int labelIndex;
	undef labelName;
public:


	// Private attribute accessor methods
	//  

private:

public:


	// Private attribute accessor methods
	//  


	/**
	 * Set the value of labelIndex
	 * @param new_var the new value of labelIndex
	 */
	void setLabelIndex (int new_var)	 {
			labelIndex = new_var;
	}

	/**
	 * Get the value of labelIndex
	 * @return the value of labelIndex
	 */
	int getLabelIndex ()	 {
		return labelIndex;
	}

	/**
	 * Set the value of labelName
	 * @param new_var the new value of labelName
	 */
	void setLabelName (undef new_var)	 {
			labelName = new_var;
	}

	/**
	 * Get the value of labelName
	 * @return the value of labelName
	 */
	undef getLabelName ()	 {
		return labelName;
	}
private:


	void initAttributes () ;

};
} // end of package namespace

#endif // OBJECTCLASSLABEL_H
