//
// Created by khuang on 9/3/18.
//
/**
\brief: declaration of global variables
*/

#pragma once

/**
 * @brief switching argument between Background and non-background, need to switch for each operation involving access of voxels and hash table
 */
extern bool sceneIsBackground;
//#pragma omp threadprivate(sceneIsBackground) //only active when using parallel track and map

extern bool saveSTL; //save STLs or not, set during program start
extern int STL_Frequency; //save STLs every X frames.
extern int reader_SkipFrames; // Set the number of frames the reader should skip
extern int numthreads;
extern int totFrames; //Set the total number of frames before stop
extern bool do_BG_cleanup; //Set if the Background cleanup will be performed
extern bool do_Obj_cleanup; //Set if the Object cleanup will be performed
extern bool do_Obj_tracking; //Set if the Object level tracking will be performed

 //MT_OBJSLAM_OBJSLAMVOXELSCENEPARAMS_H
