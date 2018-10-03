//
// Created by khuang on 9/27/18.
//

#ifndef MT_OBJSLAM_MS_READER_H
#define MT_OBJSLAM_MS_READER_H


#include "TUM_Reader.h"

class MS_Reader: protected TUM_Reader {
public:
    MS_Reader(string _path, Vector2i _imgSize):TUM_Reader(_path, _imgSize){

//        img_number=1;
//        calib_path = path+"calib.txt";
//        ReadCalib(calib_path);
//        string associate_file_name = path + "/associate.txt";
//        associate_f_stream.open(associate_file_name);
//        viewBuilder = new ITMLib::ITMViewBuilder_CPU(*calib);
//        cout<<"Created TUM_Reader Path "<<path<<endl;

    }


    RGB_D_NamePair Get_RGB_D_filenames(std::istream &associate_src){
            string depth_name;
            string rgb_name;
            string garbage;

            if(associate_src.eof()) return RGB_D_NamePair{"",""};
            associate_src>>garbage;
            associate_src>>depth_name;
            associate_src>>garbage;
            associate_src>>rgb_name;


            //skip 2 frames
            for(int i = 0; i < reader_SkipFrames; ++i){
                    associate_src>>garbage;
                    associate_src>>garbage;
                    associate_src>>garbage;
                    associate_src>>garbage;
            }

            RGB_D_NamePair res;
            res.depth_name=depth_name;
            res.rgb_name=rgb_name;
            return  res;
    }

};


#endif //MT_OBJSLAM_MS_READER_H
