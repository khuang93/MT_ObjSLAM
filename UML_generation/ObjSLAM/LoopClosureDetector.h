
#ifndef LOOPCLOSUREDETECTOR_H
#define LOOPCLOSUREDETECTOR_H

#include <string>

namespace ObjSLAM {


/**
  * class LoopClosureDetector
  * 
  */

class LoopClosureDetector
{
public:

  // Constructors/Destructors
  //  


  /**
   * Empty Constructor
   */
  LoopClosureDetector ();

  /**
   * Empty Destructor
   */
  virtual ~LoopClosureDetector ();

  // Static Public attributes
  //  

  // Public attributes
  //  


  // Public attribute accessor methods
  //  


  // Public attribute accessor methods
  //  



  /**
   */
  void detectLoop ()
  {
  }


  /**
   */
  void optimizePoseGraph ()
  {
  }


  /**
   */
  void ComputeSim3 ()
  {
  }


  /**
   */
  void optimizeObjectLocations ()
  {
  }


  /**
   * Update the reconstructed scene according to the updated camera poses
   * Fuse the reconstruction on both sides of the loop
   * 
   */
  void updateAndFuseReconstruction ()
  {
  }

protected:

  // Static Protected attributes
  //  

  // Protected attributes
  //  

  new_class_3 globalGraph;
  new_class_4 localGraph;
public:


  // Protected attribute accessor methods
  //  

protected:

public:


  // Protected attribute accessor methods
  //  


  /**
   * Set the value of globalGraph
   * @param new_var the new value of globalGraph
   */
  void setGlobalGraph (new_class_3 new_var)   {
      globalGraph = new_var;
  }

  /**
   * Get the value of globalGraph
   * @return the value of globalGraph
   */
  new_class_3 getGlobalGraph ()   {
    return globalGraph;
  }

  /**
   * Set the value of localGraph
   * @param new_var the new value of localGraph
   */
  void setLocalGraph (new_class_4 new_var)   {
      localGraph = new_var;
  }

  /**
   * Get the value of localGraph
   * @return the value of localGraph
   */
  new_class_4 getLocalGraph ()   {
    return localGraph;
  }
protected:


private:

  // Static Private attributes
  //  

  // Private attributes
  //  

  new_class_9 GlobalVBA;
public:


  // Private attribute accessor methods
  //  

private:

public:


  // Private attribute accessor methods
  //  


  /**
   * Set the value of GlobalVBA
   * @param new_var the new value of GlobalVBA
   */
  void setGlobalVBA (new_class_9 new_var)   {
      GlobalVBA = new_var;
  }

  /**
   * Get the value of GlobalVBA
   * @return the value of GlobalVBA
   */
  new_class_9 getGlobalVBA ()   {
    return GlobalVBA;
  }
private:


  void initAttributes () ;

};
} // end of package namespace

#endif // LOOPCLOSUREDETECTOR_H
