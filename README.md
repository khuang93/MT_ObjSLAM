# MT_ObjSLAM

This is the Repo of the master thesis of dense object SLAM. It is based on the dense slam algorithm InfiniTAM and will fuse the funcitonalities of Object Segmentation to reconstruct objects searately and use them to perform place recognition and loop closure.

Clone the repo with its submodules: 
```
$ git clone --recurse-submodule
```
Dependencies:
- All the InfiniTAM dependencies.
- OpenMP
- libpng++-dev (if using png files)

To make make the project:

```
$ mkdir build
$ cd build/
$ cmake .. -DWITH_FFMPEG=false -DWITH_CUDA=false -DWITH_OPENMP=false -DWITH_PNG=true
$ make
```

Download the dataset which are already prepared with the segmentation: https://polybox.ethz.ch/index.php/s/lElQPbrMxiuBT4s .

Run the project:
``` 
$ ./TrackerMapper [Path_To_Dataset]  [Frame_Number] [saveSTL?] [STL_Frequency]
```
Path_To_Dataset is the path to a dataset, for example home/living_room_traj1_frei_png/. Do not change the folder names of the dataset as the name serves as an identification of which kind of data format it has.

Frame_Number is a positiv integer number, which specifies how many frames will it run.

saveSTL = 0 is not save STL, other wise it will save STL model for each object.

STL_Frequency controls how often (every n frames) will STLs be saved.

Currently the most up-to-date code is on the branch reduceMemory use. with submodule InfiniTAM on branch reduceMemory.
