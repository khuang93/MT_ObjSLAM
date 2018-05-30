
#ifndef SEMANTICSCENE_H
#define SEMANTICSCENE_H
#include "ITMLib/ITMScene.h"

#include <string>
using ITMLib::ITMScene;

namespace ObjSLAM {


/**
  * class SemanticScene
  * 
  */

class SemanticScene : public ITMScene
{
public:

  // Constructors/Destructors
  //  


  /**
   * Empty Constructor
   */
  SemanticScene ();

  /**
   * Empty Destructor
   */
  virtual ~SemanticScene ();

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

  std::vector<ObjectInstance> listOfAllObjects;
  ObjSLAM::ObjectsGraph objectsGraph;
public:


  // Private attribute accessor methods
  //  

private:

public:


  // Private attribute accessor methods
  //  


  /**
   * Set the value of listOfAllObjects
   * @param new_var the new value of listOfAllObjects
   */
  void setListOfAllObjects (std::vector<ObjectInstance> new_var)   {
      listOfAllObjects = new_var;
  }

  /**
   * Get the value of listOfAllObjects
   * @return the value of listOfAllObjects
   */
  std::vector<ObjectInstance> getListOfAllObjects ()   {
    return listOfAllObjects;
  }

  /**
   * Set the value of objectsGraph
   * @param new_var the new value of objectsGraph
   */
  void setObjectsGraph (ObjSLAM::ObjectsGraph new_var)   {
      objectsGraph = new_var;
  }

  /**
   * Get the value of objectsGraph
   * @return the value of objectsGraph
   */
  ObjSLAM::ObjectsGraph getObjectsGraph ()   {
    return objectsGraph;
  }
private:


  void initAttributes () ;

};
} // end of package namespace

#endif // SEMANTICSCENE_H
