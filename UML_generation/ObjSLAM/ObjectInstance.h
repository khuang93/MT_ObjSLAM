
#ifndef OBJECTINSTANCE_H
#define OBJECTINSTANCE_H

#include <string>

namespace ObjSLAM {


/**
  * class ObjectInstance
  * 
  */

class ObjectInstance
{
public:

  // Constructors/Destructors
  //  


  /**
   * Empty Constructor
   */
  ObjectInstance ();

  /**
   * Empty Destructor
   */
  virtual ~ObjectInstance ();

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

  ObjSLAM::ObjectClassLabel objectClassLabel;
  std::vector<SemanticView>* inViews;
  // All voxels of this object
  // type still unclear
  // 
  new_class_2 voxelBlocks;
public:


  // Private attribute accessor methods
  //  

private:

public:


  // Private attribute accessor methods
  //  


  /**
   * Set the value of objectClassLabel
   * @param new_var the new value of objectClassLabel
   */
  void setObjectClassLabel (ObjSLAM::ObjectClassLabel new_var)   {
      objectClassLabel = new_var;
  }

  /**
   * Get the value of objectClassLabel
   * @return the value of objectClassLabel
   */
  ObjSLAM::ObjectClassLabel getObjectClassLabel ()   {
    return objectClassLabel;
  }

  /**
   * Set the value of inViews
   * @param new_var the new value of inViews
   */
  void setInViews (std::vector<SemanticView>* new_var)   {
      inViews = new_var;
  }

  /**
   * Get the value of inViews
   * @return the value of inViews
   */
  std::vector<SemanticView>* getInViews ()   {
    return inViews;
  }

  /**
   * Set the value of voxelBlocks
   * All voxels of this object
   * type still unclear
   * 
   * @param new_var the new value of voxelBlocks
   */
  void setVoxelBlocks (new_class_2 new_var)   {
      voxelBlocks = new_var;
  }

  /**
   * Get the value of voxelBlocks
   * All voxels of this object
   * type still unclear
   * 
   * @return the value of voxelBlocks
   */
  new_class_2 getVoxelBlocks ()   {
    return voxelBlocks;
  }
private:


  void initAttributes () ;

};
} // end of package namespace

#endif // OBJECTINSTANCE_H
