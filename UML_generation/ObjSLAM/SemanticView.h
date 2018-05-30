
#ifndef SEMANTICVIEW_H
#define SEMANTICVIEW_H
#include "ITMLib/ITMView.h"
#include "ITMLib/ITMView.h"

#include <string>
using ITMLib::ITMView;
using ITMLib::ITMView;

namespace ObjSLAM {


/**
  * class SemanticView
  * 
  */

class SemanticView : public ITMView, public ITMView
{
public:

  // Constructors/Destructors
  //  


  /**
   * Empty Constructor
   */
  SemanticView ();

  /**
   * Empty Destructor
   */
  virtual ~SemanticView ();

  // Static Public attributes
  //  

  // Public attributes
  //  

  ITMUIntImage segmentationImage;

  // Public attribute accessor methods
  //  


  // Public attribute accessor methods
  //  


  /**
   * Set the value of segmentationImage
   * @param new_var the new value of segmentationImage
   */
  void setSegmentationImage (ITMUIntImage new_var)   {
      segmentationImage = new_var;
  }

  /**
   * Get the value of segmentationImage
   * @return the value of segmentationImage
   */
  ITMUIntImage getSegmentationImage ()   {
    return segmentationImage;
  }


  /**
   * @return ITMUIntImage
   */
  ITMUIntImage getSegmentationMask ()
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
  void setListOfObjectsInView (std::vector< int >* new_var)   {
      listOfObjectsInView = new_var;
  }

  /**
   * Get the value of listOfObjectsInView
   * Type of the list to be discussed
   * 
   * @return the value of listOfObjectsInView
   */
  std::vector< int >* getListOfObjectsInView ()   {
    return listOfObjectsInView;
  }
private:


  void initAttributes () ;

};
} // end of package namespace

#endif // SEMANTICVIEW_H
