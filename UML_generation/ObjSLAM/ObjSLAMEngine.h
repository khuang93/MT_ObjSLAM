
#ifndef OBJSLAMENGINE_H
#define OBJSLAMENGINE_H

#include <string>

namespace ObjSLAM {


/**
  * class ObjSLAMEngine
  * 
  */

class ObjSLAMEngine
{
public:

  // Constructors/Destructors
  //  


  /**
   * Empty Constructor
   */
  ObjSLAMEngine ();

  /**
   * Empty Destructor
   */
  virtual ~ObjSLAMEngine ();

  // Static Public attributes
  //  

  // Public attributes
  //  


  // Public attribute accessor methods
  //  


  // Public attribute accessor methods
  //  



  /**
   * @return ObjSLAM::InfiniTAMCreator
   */
  ObjSLAM::InfiniTAMCreator init_InfiniTAMCreator ()
  {
  }


  /**
   */
  void init_SemanticLabelGreator ()
  {
  }


  /**
   * @return ObjSLAM::ObjectSceneReconstructionEngine
   */
  ObjSLAM::ObjectSceneReconstructionEngine init_ObjectSceneReconstructionEngine ()
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

  ObjSLAM::InfiniTAMCreator infiniTAMCreator;
  ObjSLAM::SemanticLabelCreator semanticLabelCreator;
  ObjSLAM::ObjectSceneReconstructionEngine objectSceneReconstructionEngine;
  ObjSLAM::LoopClosureDetector loopCLosureDetector;
public:


  // Private attribute accessor methods
  //  

private:

public:


  // Private attribute accessor methods
  //  


  /**
   * Set the value of infiniTAMCreator
   * @param new_var the new value of infiniTAMCreator
   */
  void setInfiniTAMCreator (ObjSLAM::InfiniTAMCreator new_var)   {
      infiniTAMCreator = new_var;
  }

  /**
   * Get the value of infiniTAMCreator
   * @return the value of infiniTAMCreator
   */
  ObjSLAM::InfiniTAMCreator getInfiniTAMCreator ()   {
    return infiniTAMCreator;
  }

  /**
   * Set the value of semanticLabelCreator
   * @param new_var the new value of semanticLabelCreator
   */
  void setSemanticLabelCreator (ObjSLAM::SemanticLabelCreator new_var)   {
      semanticLabelCreator = new_var;
  }

  /**
   * Get the value of semanticLabelCreator
   * @return the value of semanticLabelCreator
   */
  ObjSLAM::SemanticLabelCreator getSemanticLabelCreator ()   {
    return semanticLabelCreator;
  }

  /**
   * Set the value of objectSceneReconstructionEngine
   * @param new_var the new value of objectSceneReconstructionEngine
   */
  void setObjectSceneReconstructionEngine (ObjSLAM::ObjectSceneReconstructionEngine new_var)   {
      objectSceneReconstructionEngine = new_var;
  }

  /**
   * Get the value of objectSceneReconstructionEngine
   * @return the value of objectSceneReconstructionEngine
   */
  ObjSLAM::ObjectSceneReconstructionEngine getObjectSceneReconstructionEngine ()   {
    return objectSceneReconstructionEngine;
  }

  /**
   * Set the value of loopCLosureDetector
   * @param new_var the new value of loopCLosureDetector
   */
  void setLoopCLosureDetector (ObjSLAM::LoopClosureDetector new_var)   {
      loopCLosureDetector = new_var;
  }

  /**
   * Get the value of loopCLosureDetector
   * @return the value of loopCLosureDetector
   */
  ObjSLAM::LoopClosureDetector getLoopCLosureDetector ()   {
    return loopCLosureDetector;
  }
private:


  void initAttributes () ;

};
} // end of package namespace

#endif // OBJSLAMENGINE_H
