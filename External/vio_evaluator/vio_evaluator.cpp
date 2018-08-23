#include <iostream>
#include <math.h>
#include <vector>
#include <limits>
#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include "transformation/transformation.h"

typedef Eigen::VectorXd VectorXd;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Quaterniond Quaterniond;

using namespace VGUGV;

// static parameter
float lengths[] = {200,400,600,800};
int32_t num_lengths = 4;

struct errors {
  int32_t first_frame;
  float   r_err;
  float   t_err;
  float   len;
  float   speed;
  errors (int32_t first_frame,float r_err,float t_err,float len,float speed) :
          first_frame(first_frame),r_err(r_err),t_err(t_err),len(len),speed(speed) {}
};

#define FRAME_RATE 0.04
#define INDEX_KEYFRAME 1
#define START_INDEX_VO_POSE 2
#define START_INDEX_GT_POSE 9
#define START_INDEX_VO_TIME 16
#define INDEX_GT_SPEED 23

//#define IGNORE_HEIGH

std::vector<VectorXd> loadData(std::string file_name) {
  std::vector<VectorXd> data;
  FILE *fp = fopen(file_name.c_str(),"r");
  if (!fp)
    return data;

  // read out the first line
  char dummy[1024];
  fgets(dummy, 1024, fp);

  size_t pos;
  std::string line(dummy);
  size_t nEntries = 1;
  while ((pos = line.find(',')) != std::string::npos)
  {
	  nEntries++;
	  line.erase(0, pos + 1);
  }

  while (!feof(fp)) {
	  VectorXd _data = Eigen::Matrix<double, 36, 1>::Zero();
     
	unsigned long long timeStamp;
	if (nEntries == 23 && fscanf(fp,
		"%lld,%lf,"
		"%lf,%lf,%lf,%lf,%lf,%lf,%lf,"
		"%lf,%lf,%lf,%lf,%lf,%lf,%lf,"
		"%lf,%lf,%lf,"
		"%lf,%lf,%lf,%lf,%lf",
		&timeStamp, &_data(1),
		&_data(2), &_data(3), &_data(4), &_data(5), &_data(6), &_data(7), &_data(8),
		&_data(9), &_data(10), &_data(11), &_data(12), &_data(13), &_data(14), &_data(15),
		&_data(16), &_data(17), &_data(18),
		&_data(19), &_data(20), &_data(21), &_data(22)) == 23)
	{
		_data(0) = timeStamp;
		data.push_back(_data);
	}
	// make it compatible with old log files (no speed)
	else if (nEntries == 24 && fscanf(fp,
		"%lld,%lf,"
		"%lf,%lf,%lf,%lf,%lf,%lf,%lf,"
		"%lf,%lf,%lf,%lf,%lf,%lf,%lf,"
		"%lf,%lf,%lf,"
		"%lf,%lf,%lf,%lf,%lf",
		&timeStamp, &_data(1),
		&_data(2), &_data(3), &_data(4), &_data(5), &_data(6), &_data(7), &_data(8),
		&_data(9), &_data(10), &_data(11), &_data(12), &_data(13), &_data(14), &_data(15),
		&_data(16), &_data(17), &_data(18),
		&_data(19), &_data(20), &_data(21), &_data(22),
		&_data(23)) == 24)
	{
		_data(0) = timeStamp;
		data.push_back(_data);
	}
	else if (nEntries == 36 && fscanf(fp,
		"%lld,%lf,"
		"%lf,%lf,%lf,%lf,%lf,%lf,%lf,"
		"%lf,%lf,%lf,%lf,%lf,%lf,%lf,"
		"%lf,%lf,%lf,"
		"%lf,%lf,%lf,%lf,%lf,"
		"%f,"
		"%f,%f,%f,%f,%f,%f,"
		"%f,%f,%f,%f,%f,%f",
		&timeStamp, &_data(1),
		&_data(2), &_data(3), &_data(4), &_data(5), &_data(6), &_data(7), &_data(8),
		&_data(9), &_data(10), &_data(11), &_data(12), &_data(13), &_data(14), &_data(15),
		&_data(16), &_data(17), &_data(18),
		&_data(19), &_data(20), &_data(21), &_data(22), 
		&_data(23), 
		&_data(24), &_data(25), &_data(26), &_data(27), &_data(28), &_data(29), 
		&_data(30), &_data(31), &_data(32), &_data(33), &_data(34), &_data(35)) == 36)
	{
		_data(0) = timeStamp;
		data.push_back(_data);
	}
	else break;
  }
  fclose(fp);
  return data;
}

Common::Transformation get_pose(const std::vector<VectorXd>& data, int index_4_vo_gt_selection, int index_4_data)
{
	VectorXd indexed_data = data[index_4_data];

	Quaterniond R = Quaterniond(indexed_data(index_4_vo_gt_selection),
		indexed_data(index_4_vo_gt_selection + 1),
		indexed_data(index_4_vo_gt_selection + 2),
		indexed_data(index_4_vo_gt_selection + 3));
	R.normalize();

	Vector3d t = indexed_data.block(index_4_vo_gt_selection + 4, 0, 3, 1);
	Common::Transformation T(R, t);
	return T;
}

std::vector<float> trajectoryDistances (const std::vector<VectorXd>& data) {
  std::vector<float> dist;
  dist.push_back(0);
  for (int32_t i = 1; i < data.size(); i++) {
    Vector3d P1 = data[i-1].block(START_INDEX_VO_POSE + 4, 0, 3, 1);
    Vector3d P2 = data[i].block(START_INDEX_VO_POSE + 4, 0, 3, 1);
    dist.push_back(dist[i-1] + (P1 - P2).norm());
  }

  std::cout << __FUNCTION__ << " total trajectory length: " << dist.back() << "\n";
  return dist;
}


int32_t lastFrameFromSegmentLength(std::vector<float>& dist, int32_t first_frame, float len) {
  for (int32_t i = first_frame; i< dist.size(); i++)
    if (dist[i]>dist[first_frame]+len)
      return i;
  return -1;
}

std::vector<errors> calcSequenceErrors (const std::vector<VectorXd>& data) {

  printf("Got %lu poses\n", data.size());

  // error vector
  std::vector<errors> err;

  // parameters
  int32_t step_size = 10; // every second

  // pre-compute distances (from ground truth as reference)
  std::vector<float> dist = trajectoryDistances(data);

  // for all start positions do
  for (int32_t first_frame = 0; first_frame < data.size(); first_frame += step_size) {

    // for all segment lengths do
    for (int32_t i = 0; i < num_lengths; i++) {

      // current length
      float len = lengths[i];
      // compute last frame
      int32_t last_frame = lastFrameFromSegmentLength(dist, first_frame, len);

      // continue, if sequence not long enough
      if (last_frame==-1)
        continue;

      // compute rotational and translational errors
      VectorXd _data_0 = data[first_frame];
      VectorXd _data_1 = data[last_frame];

      // gt
      Quaterniond gt_R_0 = Quaterniond(_data_0(START_INDEX_GT_POSE),
                                       _data_0(START_INDEX_GT_POSE + 1),
                                       _data_0(START_INDEX_GT_POSE + 2),
                                       _data_0(START_INDEX_GT_POSE + 3));
      gt_R_0.normalize();
      Vector3d gt_t_0 = _data_0.block(START_INDEX_GT_POSE + 4, 0, 3, 1);

#ifdef IGNORE_HEIGH
        gt_t_0(2) = 0.0;
#endif

      VGUGV::Common::Transformation T_gt_0(gt_R_0, gt_t_0);

      Quaterniond gt_R_1 = Quaterniond(_data_1(START_INDEX_GT_POSE),
                                       _data_1(START_INDEX_GT_POSE + 1),
                                       _data_1(START_INDEX_GT_POSE + 2),
                                       _data_1(START_INDEX_GT_POSE + 3));
      Vector3d gt_t_1 = _data_1.block(START_INDEX_GT_POSE + 4, 0, 3, 1);

#ifdef IGNORE_HEIGH
        gt_t_1(2) = 0.0;
#endif

      gt_R_1.normalize();
      VGUGV::Common::Transformation T_gt_1(gt_R_1, gt_t_1);

      // vo
      Quaterniond vo_R_0 = Quaterniond(_data_0(START_INDEX_VO_POSE),
                                       _data_0(START_INDEX_VO_POSE + 1),
                                       _data_0(START_INDEX_VO_POSE + 2),
                                       _data_0(START_INDEX_VO_POSE + 3));
      vo_R_0.normalize();
      Vector3d vo_t_0 = _data_0.block(START_INDEX_VO_POSE + 4, 0, 3, 1);

#ifdef IGNORE_HEIGH
        vo_t_0(2) = 0.0;
#endif

      VGUGV::Common::Transformation T_vo_0(vo_R_0, vo_t_0);

      Quaterniond vo_R_1 = Quaterniond(_data_1(START_INDEX_VO_POSE),
                                       _data_1(START_INDEX_VO_POSE + 1),
                                       _data_1(START_INDEX_VO_POSE + 2),
                                       _data_1(START_INDEX_VO_POSE + 3));
      vo_R_1.normalize();
      Vector3d vo_t_1 = _data_1.block(START_INDEX_VO_POSE + 4, 0, 3, 1);

#ifdef IGNORE_HEIGH
        vo_t_1(2) = 0.0;
#endif

      VGUGV::Common::Transformation T_vo_1(vo_R_1, vo_t_1);

      // compute error
      VGUGV::Common::Transformation delta_T_gt = T_gt_1.inverse() * T_gt_0;
      VGUGV::Common::Transformation delta_T_vo = T_vo_1.inverse() * T_vo_0;
      VGUGV::Common::Transformation delta_error = delta_T_gt.inverse() * delta_T_vo;

      Eigen::Vector3d rpy = delta_error.getRollPitchYaw();
      float r_err = rpy.norm();
      float t_err = delta_error.translation().norm();

      // compute speed
      float num_frames = (float)(last_frame-first_frame+1);
      float speed = len/(FRAME_RATE * num_frames); // in m/s

      // write to file
      err.push_back(errors(first_frame, r_err / len, t_err / len,len, speed));
    }
  }
  // return error vector
  return err;
}


void saveSequenceErrors (std::vector<errors> &err, std::string file_name)
{
  printf("Write %lu sequence errors to %s\n", err.size(), file_name.c_str());

  // open file
  FILE *fp;
  fp = fopen(file_name.c_str(),"w");

  // write to file
  for (std::vector<errors>::iterator it=err.begin(); it!=err.end(); it++) {
    fprintf(fp,"%d %f %f %f %f\n",it->first_frame,it->r_err,it->t_err,it->len,it->speed);
  }
  // close file
  fclose(fp);
}

void savePathPlot (const std::vector<VectorXd>& data, std::string file_name) {
  // parameters
  int32_t step_size = 3;

  printf("Save path plot\n");
  // open file
  FILE *fp = fopen(file_name.c_str(),"w");

  // save x/y coordinates of all frames to file
  for (int32_t i=0; i < data.size(); i+=step_size) {
      VectorXd _data = data[i];
      const float gt_x = _data(START_INDEX_GT_POSE + 4);
      const float gt_y = _data(START_INDEX_GT_POSE + 5);

      const float vo_x = _data(START_INDEX_VO_POSE + 4);
      const float vo_y = _data(START_INDEX_VO_POSE + 5);
      fprintf(fp,"%f %f %f %f\n",gt_x, gt_y, vo_x, vo_y);
  }
  // close file
  fclose(fp);
}

std::vector<int32_t> computeRoi (const std::vector<VectorXd>& data) {
    float x_min = std::numeric_limits<int32_t>::max();
    float x_max = std::numeric_limits<int32_t>::min();
    float y_min = std::numeric_limits<int32_t>::max();
    float y_max = std::numeric_limits<int32_t>::min();

    for(auto _data : data){
        const float gt_x = _data(START_INDEX_GT_POSE + 4);
        const float gt_y = _data(START_INDEX_GT_POSE + 5);

        const float vo_x = _data(START_INDEX_VO_POSE + 4);
        const float vo_y = _data(START_INDEX_VO_POSE + 5);

        if (gt_x < x_min) x_min = gt_x;
        if (gt_x > x_max) x_max = gt_x;
        if (gt_y < y_min) y_min = gt_y;
        if (gt_y > y_max) y_max = gt_y;

        if (vo_x < x_min) x_min = vo_x;
        if (vo_x > x_max) x_max = vo_x;
        if (vo_y < y_min) y_min = vo_y;
        if (vo_y > y_max) y_max = vo_y;
    }

    float dx = 1.1 * (x_max - x_min);
    float dy = 1.1 * (y_max - y_min);
    float mx = 0.5 * (x_max + x_min);
    float mz = 0.5 * (y_max + y_min);
    float r = 0.5 * std::max(dx, dy);

    std::vector <int32_t> roi;
    roi.push_back((int32_t) (mx - r));
    roi.push_back((int32_t) (mx + r));
    roi.push_back((int32_t) (mz - r));
    roi.push_back((int32_t) (mz + r));
    return roi;
}

void plotPathPlot (std::string dir, std::vector<int32_t> &roi, std::string filename) {

  // gnuplot file name
  char command[1024];
  char file_name[256];
  sprintf(file_name,"%s.gp",filename.c_str());
  std::string full_name = dir + "/" + file_name;

  // create png + eps
  for (int32_t i=0; i<2; i++) {

    // open file
    FILE *fp = fopen(full_name.c_str(),"w");

    // save gnuplot instructions
    if (i==0) {
      fprintf(fp,"set term png size 900,900\n");
      fprintf(fp,"set output \"%s.png\"\n",filename.c_str());
    } else {
      fprintf(fp,"set term postscript eps enhanced color\n");
      fprintf(fp,"set output \"%s.eps\"\n",filename.c_str());
    }

    fprintf(fp, "set key font \",18\"\n");
    fprintf(fp,"set size ratio -1\n");
    fprintf(fp,"set xrange [%d:%d]\n", roi[0], roi[1]);
    fprintf(fp,"set yrange [%d:%d]\n", roi[2], roi[3]);
    fprintf(fp,"set xlabel \"x [m]\"\n");
    fprintf(fp,"set ylabel \"y [m]\"\n");
    fprintf(fp,"plot \"%s.txt\" using 1:2 lc rgb \"#0000FF\" title 'Ground Truth' w lines,",filename.c_str());
    fprintf(fp,"\"%s.txt\" using 3:4 lc rgb \"#FF0000\" title 'Visual Odometry' w lines,",filename.c_str());
    fprintf(fp,"\"< head -1 %s.txt\" using 1:2 lc rgb \"#000000\" pt 4 ps 1 lw 2 title 'Sequence Start' w points\n",filename.c_str());

    // close file
    fclose(fp);
    // run gnuplot => create png + eps
    sprintf(command,"cd %s; gnuplot %s",dir.c_str(),file_name);
    system(command);
  }

  // create pdf and crop
  sprintf(command,"cd %s; ps2pdf %s.eps %s_large.pdf",dir.c_str(),filename.c_str(),filename.c_str());
  system(command);
  sprintf(command,"cd %s; pdfcrop %s_large.pdf %s.pdf",dir.c_str(),filename.c_str(),filename.c_str());
  system(command);
  sprintf(command,"cd %s; rm %s_large.pdf",dir.c_str(),filename.c_str());
  system(command);
}

void saveErrorPlots(std::vector<errors> &seq_err, std::string plot_error_dir,char* prefix) {

  // file names
  char file_name_tl[1024]; sprintf(file_name_tl,"%s/%s_tl.txt",plot_error_dir.c_str(),prefix);
  char file_name_rl[1024]; sprintf(file_name_rl,"%s/%s_rl.txt",plot_error_dir.c_str(),prefix);
  char file_name_ts[1024]; sprintf(file_name_ts,"%s/%s_ts.txt",plot_error_dir.c_str(),prefix);
  char file_name_rs[1024]; sprintf(file_name_rs,"%s/%s_rs.txt",plot_error_dir.c_str(),prefix);

  // open files
  FILE *fp_tl = fopen(file_name_tl,"w");
  FILE *fp_rl = fopen(file_name_rl,"w");
  FILE *fp_ts = fopen(file_name_ts,"w");
  FILE *fp_rs = fopen(file_name_rs,"w");

  // for each segment length do
  for (int32_t i=0; i<num_lengths; i++) {

    float t_err = 0;
    float r_err = 0;
    float num   = 0;

    // for all errors do
    for (std::vector<errors>::iterator it=seq_err.begin(); it!=seq_err.end(); it++) {
      if (fabs(it->len-lengths[i])<1.0) {
        t_err += it->t_err;
        r_err += it->r_err;
        num++;
      }
    }

    // we require at least 3 values
    if (num>2.5) {
      fprintf(fp_tl,"%f %f\n",lengths[i],t_err/num);
      fprintf(fp_rl,"%f %f\n",lengths[i],r_err/num);
    }
  }

  // for each driving speed do (in m/s)
  for (float speed=2; speed<25; speed+=2) {

    float t_err = 0;
    float r_err = 0;
    float num   = 0;

    // for all errors do
    for (std::vector<errors>::iterator it=seq_err.begin(); it!=seq_err.end(); it++) {
      if (fabs(it->speed-speed)<2.0) {
        t_err += it->t_err;
        r_err += it->r_err;
        num++;
      }
    }

    // we require at least 3 values
    if (num>2.5) {
      fprintf(fp_ts,"%f %f\n",speed,t_err/num);
      fprintf(fp_rs,"%f %f\n",speed,r_err/num);
    }
  }

  // close files
  fclose(fp_tl);
  fclose(fp_rl);
  fclose(fp_ts);
  fclose(fp_rs);
}

void plotErrorPlots (std::string dir,char* prefix) {

  char command[1024];

  // for all four error plots do
  for (int32_t i=0; i<4; i++) {

    // create suffix
    char suffix[16];
    switch (i) {
      case 0: sprintf(suffix,"tl"); break;
      case 1: sprintf(suffix,"rl"); break;
      case 2: sprintf(suffix,"ts"); break;
      case 3: sprintf(suffix,"rs"); break;
    }

    // gnuplot file name
    char file_name[1024]; char full_name[1024];
    sprintf(file_name,"%s_%s.gp",prefix,suffix);
    sprintf(full_name,"%s/%s",dir.c_str(),file_name);

    // create png + eps
    for (int32_t j=0; j<2; j++) {

      // open file
      FILE *fp = fopen(full_name,"w");

      // save gnuplot instructions
      if (j==0) {
        fprintf(fp,"set term png size 500,250 font \"Helvetica\" 11\n");
        fprintf(fp,"set output \"%s_%s.png\"\n",prefix,suffix);
      } else {
        fprintf(fp,"set term postscript eps enhanced color\n");
        fprintf(fp,"set output \"%s_%s.eps\"\n",prefix,suffix);
      }

      // start plot at 0
      fprintf(fp,"set size ratio 0.5\n");
      fprintf(fp,"set yrange [0:*]\n");

      // x label
      if (i<=1) fprintf(fp,"set xlabel \"Path Length [m]\"\n");
      else      fprintf(fp,"set xlabel \"Speed [km/h]\"\n");

      // y label
      if (i==0 || i==2) fprintf(fp,"set ylabel \"Translation Error [%%]\"\n");
      else              fprintf(fp,"set ylabel \"Rotation Error [deg/m]\"\n");

      // plot error curve
      fprintf(fp,"plot \"%s_%s.txt\" using ",prefix,suffix);
      switch (i) {
        case 0: fprintf(fp,"1:($2*100) title 'Translation Error'"); break;
        case 1: fprintf(fp,"1:($2*57.3) title 'Rotation Error'"); break;
        case 2: fprintf(fp,"($1*3.6):($2*100) title 'Translation Error'"); break;
        case 3: fprintf(fp,"($1*3.6):($2*57.3) title 'Rotation Error'"); break;
      }
      fprintf(fp," lc rgb \"#0000FF\" pt 4 w linespoints\n");

      // close file
      fclose(fp);

      // run gnuplot => create png + eps
      sprintf(command,"cd %s; gnuplot %s",dir.c_str(),file_name);
      system(command);
    }

    // create pdf and crop
    sprintf(command,"cd %s; ps2pdf %s_%s.eps %s_%s_large.pdf",dir.c_str(),prefix,suffix,prefix,suffix);
    system(command);
    sprintf(command,"cd %s; pdfcrop %s_%s_large.pdf %s_%s.pdf",dir.c_str(),prefix,suffix,prefix,suffix);
    system(command);
    sprintf(command,"cd %s; rm %s_%s_large.pdf",dir.c_str(),prefix,suffix);
    system(command);
  }
}

void saveStats (std::vector<errors> err, std::string dir) {

  float t_err = 0;
  float r_err = 0;

  // for all errors do => compute sum of t_err, r_err
  for (std::vector<errors>::iterator it=err.begin(); it!=err.end(); it++) {
    t_err += it->t_err;
    r_err += it->r_err;
  }

  // open file
  FILE *fp = fopen((dir + "/avg_drift.txt").c_str(),"w");

  // save errors
  float num = err.size();
  fprintf(fp,"avg_error: %f%% %f\n",t_err/num * 100,r_err/num);

  // close file
  fclose(fp);
}

void saveTimeStat(const std::vector<VectorXd>& data, std::string dir) {
    FILE *fpTracker = fopen((dir + "/tracker.txt").c_str(), "w");
    FILE *fpMapper  = fopen((dir + "/mapper.txt").c_str(), "w");

    double t0 = data[0](0);
    std::cout << "t0: " << t0 << "\n";
    for(int i = 0; i < data.size(); i++){
        fprintf(fpTracker, "%f %f\n", (data[i](0) - t0) * 1e-9, data[i](START_INDEX_VO_TIME + 2));
        fprintf(fpMapper, "%f %f\n", (data[i](0) - t0) * 1e-9, data[i](START_INDEX_VO_TIME + 6));
    }
    fclose(fpTracker);
    fclose(fpMapper);
}

void plotTime(std::string dir, std::string filenameWoSuffix){

    // gnuplot file name
    char command[1024];
    char file_name[256];
    sprintf(file_name,"%s.gp",filenameWoSuffix.c_str());
    std::string full_name = dir + "/" + file_name;

    // create png + eps
    for (int32_t i=0; i<2; i++) {

        // open file
        FILE *fp = fopen(full_name.c_str(),"w");

        // save gnuplot instructions
        if (i==0) {
            fprintf(fp,"set term png size 900,900\n");
            fprintf(fp,"set output \"%s.png\"\n",filenameWoSuffix.c_str());
        } else {
            fprintf(fp,"set term postscript eps enhanced color\n");
            fprintf(fp,"set output \"%s.eps\"\n",filenameWoSuffix.c_str());
        }

        fprintf(fp,"set autoscale xy\n");
        fprintf(fp,"set xlabel \"x [s]\"\n");
        fprintf(fp,"set ylabel \"y [ms]\"\n");
        fprintf(fp,"plot \"%s.txt\" using ",filenameWoSuffix.c_str());
        fprintf(fp,"1:2 title 'Time Consumption (%s)'", filenameWoSuffix.c_str());
        fprintf(fp," lc rgb \"#0000FF\" pt 4 w linespoints\n");
        // close file
        fclose(fp);
        // run gnuplot => create png + eps
        sprintf(command,"cd %s; gnuplot %s",dir.c_str(),file_name);
        system(command);
    }

    // create pdf and crop
    sprintf(command,"cd %s; ps2pdf %s.eps %s_large.pdf",dir.c_str(),filenameWoSuffix.c_str(),filenameWoSuffix.c_str());
    system(command);
    sprintf(command,"cd %s; pdfcrop %s_large.pdf %s.pdf",dir.c_str(),filenameWoSuffix.c_str(),filenameWoSuffix.c_str());
    system(command);
    sprintf(command,"cd %s; rm %s_large.pdf",dir.c_str(),filenameWoSuffix.c_str());
    system(command);
}

void plot_relative_error(const std::vector<VectorXd>& data, std::string dir)
{
	// create data file
	std::string fileNameWoSuffix("relativeError");
	FILE *fRelativeError = fopen((dir + "/" + fileNameWoSuffix + ".txt").c_str(), "w");
	for (int i = 1; i < data.size(); i++)
	{
		Common::Transformation vo_prev = get_pose(data, START_INDEX_VO_POSE, i - 1);
		Common::Transformation vo_curr = get_pose(data, START_INDEX_VO_POSE, i);
		Common::Transformation d_vo = vo_prev.inverse() * vo_curr;

		Common::Transformation gt_prev = get_pose(data, START_INDEX_GT_POSE, i - 1);
		Common::Transformation gt_curr = get_pose(data, START_INDEX_GT_POSE, i);
		Common::Transformation d_gt = gt_prev.inverse() * gt_curr;

		Common::Transformation error = d_vo * d_gt.inverse();

		Eigen::Vector3d rpy = error.getRollPitchYaw();
		Eigen::Vector3d t = error.translation();

		fprintf(fRelativeError, "%d %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
			i,
			data[i](INDEX_KEYFRAME),
			fabs(rpy(0) * 180.0 / M_PI),
			fabs(rpy(1) * 180.0 / M_PI),
			fabs(rpy(2) * 180.0 / M_PI),
			fabs(t(0)),
			fabs(t(1)),
			fabs(t(2)),
            sqrt(rpy(0) * rpy(0) + rpy(1) * rpy(1) + rpy(2) * rpy(2)) * 180.0 / M_PI,
            t.norm(),
            vo_curr.translation()(0),
            vo_curr.translation()(1),
			gt_curr.translation()(0),
			gt_curr.translation()(1),
			data[i](INDEX_GT_SPEED));
	}
	fclose(fRelativeError);

	// plot
	// create png + eps
	// open file
	// save gnuplot instructions
	for (int i = 3; i < 9; i++)
	{
		std::string gpFileNameWoSuffix;
		switch (i)
		{
		case 3:
			gpFileNameWoSuffix = "roll";
			break;
		case 4:
			gpFileNameWoSuffix = "pitch";
			break;
		case 5:
			gpFileNameWoSuffix = "yaw";
			break;
		case 6:
			gpFileNameWoSuffix = "x";
			break;
		case 7:
			gpFileNameWoSuffix = "y";
			break;
		case 8:
			gpFileNameWoSuffix = "z";
			break;
		}

		char command[1024];
		std::string fullNameGp = dir + "/" + gpFileNameWoSuffix + ".gp";

        for(int j = 0; j < 2; j++) {
            FILE *fp = fopen(fullNameGp.c_str(), "w");
            // save gnuplot instructions
            if (j==0)
            {
                fprintf(fp,"set term png size 900,900\n");
                fprintf(fp,"set output \"%s.png\"\n",gpFileNameWoSuffix.c_str());
            } else
            {
                fprintf(fp,"set term postscript eps enhanced color\n");
                fprintf(fp,"set output \"%s.eps\"\n",gpFileNameWoSuffix.c_str());
            }

            fprintf(fp, "set autoscale xy\n");
            fprintf(fp, "set xlabel \"x [frameId]\"\n");
            fprintf(fp, "set ylabel \"y [error]\"\n");
            fprintf(fp, "plot \"%s.txt\" using ", fileNameWoSuffix.c_str());
            fprintf(fp, "1:%d title 'RelativeError (%s)'", i, gpFileNameWoSuffix.c_str());
            fprintf(fp, " lc rgb \"#0000FF\" pt 4 w linespoints\n");
            // close file
            fclose(fp);
            // run gnuplot => create png + eps
            sprintf(command, "cd %s; gnuplot %s.gp", dir.c_str(), gpFileNameWoSuffix.c_str());
            system(command);
        }

		// create pdf and crop
		sprintf(command, "cd %s; ps2pdf %s.eps %s_large.pdf", dir.c_str(), gpFileNameWoSuffix.c_str(), gpFileNameWoSuffix.c_str());
		system(command);
		sprintf(command, "cd %s; pdfcrop %s_large.pdf %s.pdf", dir.c_str(), gpFileNameWoSuffix.c_str(), gpFileNameWoSuffix.c_str());
		system(command);
		sprintf(command, "cd %s; rm %s_large.pdf", dir.c_str(), gpFileNameWoSuffix.c_str());
		system(command);
	}
}

Eigen::Vector2d closeLoopDrift(const std::vector<VectorXd>& data, std::string dir)
{
    // compute rotational and translational errors
    VectorXd data_last = data.back();

    // gt
    Quaterniond gt_R_0 = Quaterniond(data_last(START_INDEX_GT_POSE),
                                     data_last(START_INDEX_GT_POSE + 1),
                                     data_last(START_INDEX_GT_POSE + 2),
                                     data_last(START_INDEX_GT_POSE + 3));
    gt_R_0.normalize();
    Vector3d gt_t_0 = data_last.block(START_INDEX_GT_POSE + 4, 0, 3, 1);

#ifdef IGNORE_HEIGH
    gt_t_0(2) = 0.0;
#endif

    VGUGV::Common::Transformation T_gt_0(gt_R_0, gt_t_0);

    // vo
    Quaterniond vo_R_0 = Quaterniond(data_last(START_INDEX_VO_POSE),
                                     data_last(START_INDEX_VO_POSE + 1),
                                     data_last(START_INDEX_VO_POSE + 2),
                                     data_last(START_INDEX_VO_POSE + 3));
    vo_R_0.normalize();
    Vector3d vo_t_0 = data_last.block(START_INDEX_VO_POSE + 4, 0, 3, 1);

#ifdef IGNORE_HEIGH
    vo_t_0(2) = 0.0;
#endif

    VGUGV::Common::Transformation T_vo_0(vo_R_0, vo_t_0);

    // compute error
    VGUGV::Common::Transformation delta_error = T_gt_0.inverse() * T_vo_0;

    Eigen::Vector3d rpy = delta_error.getRollPitchYaw();
    float r_err = rpy.norm();
    float t_err = delta_error.translation().norm();

    std::vector<float> dist = trajectoryDistances(data);

    Eigen::Vector2d drift;
    drift(0) = t_err / dist.back();
    drift(1) = (r_err * 180.0 / M_PI) / dist.back();

    std::cout << __FUNCTION__ << " close-loop drift: " << drift(0) * 100 << "% and " << drift(1) << " deg/m\n";

    // open file
    FILE *fp = fopen((dir + "/close_loop_drift.txt").c_str(),"w");

    // save errors
    fprintf(fp,"closed-loop drift: %f%% %f\n", drift(0) * 100, drift(1));

    // close file
    fclose(fp);

    return drift;
}

bool eval (const std::vector<VectorXd>& data, std::string eval_path) {
    // check for errors
    if (data.size() == 0) {
        return false;
    }

    // ground truth and result directories
    std::string error_dir = eval_path + "/errors";
    std::string plot_path_dir = eval_path + "/plot_path";
    std::string plot_error_dir = eval_path + "/plot_error";
    std::string plot_time_dir = eval_path + "/time";
	std::string plot_rel_error_dir = eval_path + "/plot_relative_error";

    // create output directories
	if (!boost::filesystem::create_directory(boost::filesystem::path(error_dir)))
	{
		std::cerr << "fails to create directory " << error_dir << "\n";
	}

	if (!boost::filesystem::create_directory(boost::filesystem::path(plot_path_dir)))
	{
		std::cerr << "fails to create directory " << plot_path_dir << "\n";
	}

	if (!boost::filesystem::create_directory(boost::filesystem::path(plot_error_dir)))
	{
		std::cerr << "fails to create directory " << plot_error_dir << "\n";
	}

	if (!boost::filesystem::create_directory(boost::filesystem::path(plot_time_dir)))
	{
		std::cerr << "fails to create directory " << plot_time_dir << "\n";
	}

	if (!boost::filesystem::create_directory(boost::filesystem::path(plot_rel_error_dir)))
	{
		std::cerr << "fails to create directory " << plot_rel_error_dir << "\n";
	}

    // compute sequence errors
    std::vector<errors> seq_err = calcSequenceErrors(data);
    saveSequenceErrors(seq_err, error_dir + "/sequenceErrors.txt");

    // add to total errors
    // total errors
    std::vector<errors> total_err;
    total_err.insert(total_err.end(), seq_err.begin(), seq_err.end());

    // save + plot bird's eye view trajectories
    savePathPlot(data, plot_path_dir + "/path.txt");
    std::vector<int32_t> roi = computeRoi(data);
    plotPathPlot(plot_path_dir, roi, "path");

    // save + plot individual errors
    char prefix[8] = "error";
    saveErrorPlots(seq_err, plot_error_dir, prefix);
    plotErrorPlots(plot_error_dir, prefix);

    // save + plot total errors + summary statistics
    if (total_err.size() > 1) {
        saveStats(total_err, eval_path);
    }

    // save time statics
    saveTimeStat(data, plot_time_dir);
    plotTime(plot_time_dir, "mapper");
    plotTime(plot_time_dir, "tracker");

    closeLoopDrift(data, eval_path);

	// plot relative error plots
	plot_relative_error(data, plot_rel_error_dir);

    // success
    return true;
}

int32_t main (int32_t argc,char *argv[]) {
    // we need 3 or 4 arguments!
    if (argc != 2 && argc != 3) {
        std::cout << "Usage: ./eval_odometry log_file_path, [eval_folder_path]" << std::endl;
        return 1;
    }

    // read arguments
    std::string log_data_file_path = argv[1];
    std::string eval_path;
    if (argc == 2) {
        eval_path = ".";
    } else {
        eval_path = argv[2];
    }

    // load in data
    std::vector<VectorXd> data = loadData(log_data_file_path);

    // run evaluation
    bool success = false;
    success = eval(data, eval_path);

    return success;
}

