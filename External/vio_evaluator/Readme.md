##Evaluation

This is an adapted version of the KITTI odometry evaluation. (http://www.cvlibs.net/datasets/kitti/eval_odometry.php).

###Usage
./evaluator log.txt [output_folder]

The pose files are expected to be csv files with one pose per line as
```ts, bKF, vo_qw, vo_qx, vo_qy, vo_qz, vo_x, vo_y, vo_z, gt_qw, gt_qx, gt_qy, gt_qz, gt_x, gt_y, gt_z, t_trackerPreprocess, t_trackerTracking, t_trackerTotal, t_mapperStereo, t_mapperResidual, t_mapperOptimize, t_mapperTotal```
and the labels in the first line.

If no output path is given, the output is written in the current folder.

###Dependencies
gnuplot http://www.gnuplot.info/