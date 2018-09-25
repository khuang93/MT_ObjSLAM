//
// Created by khuang on 9/24/18.
//

#include "ObjSLAMUI.h"


namespace ObjSLAM {

    void ObjSLAMUI::ProcessFrame() {
        if (imgNum <= totFrames) {
            sceneIsBackground = true;
            imgNum = mainEngine->readNext();
            mainEngine->trackFrame();
            mainEngine->updateMappingEngine();
            mainEngine->mapFrame();
            mainEngine->outputPics();

        }
    }


    void setImageData(unsigned char *imageArray, int size) {
        for (int i = 0; i < size; i++) {
            imageArray[i] = (unsigned char) (rand() / (RAND_MAX / 255.0));
        }
    }

    void ObjSLAMUI::CreateDisplay() {
        // Create OpenGL window in single line
        pangolin::CreateWindowAndBind("ObjSLAM UI", w, h);
    }




    void ObjSLAMUI::chooseNextObj(){
        cout<<"Object Number: \n";
//        cout<< ++(currentObjNum)<<endl;
    }

    void ObjSLAMUI::reg(){
        pangolin::RegisterKeyPressCallback(pangolin::PANGO_KEY_DOWN, [this](){chooseNextObj();} );
    }

    void ObjSLAMUI::run() {

        CreateDisplay();

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);


        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000),
                pangolin::ModelViewLookAt(-1, 1, -1, 0, 0, 0, pangolin::AxisY)
        );


        reg();



//        pangolin::View &d_cam = pangolin::Display("cam")
//                .SetBounds(0, 1.0f, 0, 1.0f, -640 / 480.0)
//                .SetHandler(new pangolin::Handler3D(s_cam));

        // This view will take up no more than a third of the windows width or height, and it
        // will have a fixed aspect ratio to match the image that it will display. When fitting
        // within the specified bounds, push to the top-left (as specified by SetLock).
//        pangolin::View &d_image = pangolin::Display("image")
//                .SetBounds(1 / 2.0f, 1.0f, 0, 1 / 2.0f, 640.0 / 480)
//                .SetLock(pangolin::LockLeft, pangolin::LockTop);
//        pangolin::View &d_image2 = pangolin::Display("image")
//                .SetBounds(1 / 2.0f, 1.0f, 1 / 2.0f, 1.0f, 640.0 / 480)
//                .SetLock(pangolin::LockLeft, pangolin::LockTop);
        pangolin::View &d_image_BG = pangolin::Display("image")
                .SetAspect(640.0f/480.0f);
        pangolin::View &d_image_obj = pangolin::Display("image2")
                .SetAspect(640.0f/480.0f);


        pangolin::Display("multi")
                .SetBounds(0.0, 1.0, 0.0, 1.0)
                .SetLayout(pangolin::LayoutEqual)
                .AddDisplay(d_image_BG)
                .AddDisplay(d_image_obj);



        std::cout << "Resize the window to experiment with SetBounds, SetLock and SetAspect." << std::endl;

            ProcessFrame();
            while (imgNum <= totFrames && !pangolin::ShouldQuit()) {


                auto *itmImage_BG = mainEngine->getImage(0);
                unsigned char *image_BG = new unsigned char[itmImage_BG->noDims.x * itmImage_BG->noDims.y * 3];

                ORUtils::Vector2<int> noDims = itmImage_BG->noDims;

                for (int i = 0; i < noDims.x * noDims.y; ++i) {
                    image_BG[i * 3 + 0] = itmImage_BG->GetData(MEMORYDEVICE_CPU)[i].x;
                    image_BG[i * 3 + 1] = itmImage_BG->GetData(MEMORYDEVICE_CPU)[i].y;
                    image_BG[i * 3 + 2] = itmImage_BG->GetData(MEMORYDEVICE_CPU)[i].z;
                }
                pangolin::GlTexture imageTexture_BG(itmImage_BG->noDims.x, itmImage_BG->noDims.y, GL_RGB, false, 0, GL_RGB,
                                                 GL_UNSIGNED_BYTE);

                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


                imageTexture_BG.Upload(image_BG, GL_RGB, GL_UNSIGNED_BYTE);


                auto *itmImage_obj = mainEngine->getImage(currentObjNum);
                unsigned char *image_obj = new unsigned char[itmImage_obj->noDims.x * itmImage_obj->noDims.y * 3];


                for (int i = 0; i < noDims.x * noDims.y; ++i) {
                    image_obj[i * 3 + 0] = itmImage_obj->GetData(MEMORYDEVICE_CPU)[i].x;
                    image_obj[i * 3 + 1] = itmImage_obj->GetData(MEMORYDEVICE_CPU)[i].y;
                    image_obj[i * 3 + 2] = itmImage_obj->GetData(MEMORYDEVICE_CPU)[i].z;
                }
                pangolin::GlTexture imageTexture_obj(itmImage_BG->noDims.x, itmImage_obj->noDims.y, GL_RGB, false, 0, GL_RGB,
                                                    GL_UNSIGNED_BYTE);

                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                imageTexture_obj.Upload(image_obj, GL_RGB, GL_UNSIGNED_BYTE);

                d_image_BG.Activate();
                glColor3f(1.0, 1.0, 1.0);
                imageTexture_BG.RenderToViewport(true);

                d_image_obj.Activate();
                glColor3f(1.0, 1.0, 1.0);
                imageTexture_obj.RenderToViewport(true);



                pangolin::FinishFrame();


                ProcessFrame();



                /*auto *itmImage = mainEngine->getImage(0);
                unsigned char *image = new unsigned char[itmImage->noDims.x * itmImage->noDims.y * 3];

                ORUtils::Vector2<int> noDims = itmImage->noDims;

                for (int i = 0; i < noDims.x * noDims.y; ++i) {
                    image[i * 3 + 0] = itmImage->GetData(MEMORYDEVICE_CPU)[i].x;
                    image[i * 3 + 1] = itmImage->GetData(MEMORYDEVICE_CPU)[i].y;
                    image[i * 3 + 2] = itmImage->GetData(MEMORYDEVICE_CPU)[i].z;
                }
                pangolin::GlTexture imageTexture(itmImage->noDims.x, itmImage->noDims.y, GL_RGB, false, 0, GL_RGB,
                                                 GL_UNSIGNED_BYTE);

                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


                imageTexture.Upload(image, GL_RGB, GL_UNSIGNED_BYTE);

                itmImage = mainEngine->getImage(1);
                unsigned char *image2 = new unsigned char[itmImage->noDims.x * itmImage->noDims.y * 3];

                noDims = itmImage->noDims;

                for (int i = 0; i < noDims.x * noDims.y; ++i) {
                    image2[i * 3 + 0] = itmImage->GetData(MEMORYDEVICE_CPU)[i].x;
                    image2[i * 3 + 1] = itmImage->GetData(MEMORYDEVICE_CPU)[i].y;
                    image2[i * 3 + 2] = itmImage->GetData(MEMORYDEVICE_CPU)[i].z;
                }
                pangolin::GlTexture imageTexture2(itmImage->noDims.x, itmImage->noDims.y, GL_RGB, false, 0, GL_RGB,
                                                 GL_UNSIGNED_BYTE);

                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                imageTexture2.Upload(image2, GL_RGB, GL_UNSIGNED_BYTE);

                itmImage = mainEngine->getImage(2);
                unsigned char *image3 = new unsigned char[itmImage->noDims.x * itmImage->noDims.y * 3];

                noDims = itmImage->noDims;

                for (int i = 0; i < noDims.x * noDims.y; ++i) {
                    image3[i * 3 + 0] = itmImage->GetData(MEMORYDEVICE_CPU)[i].x;
                    image3[i * 3 + 1] = itmImage->GetData(MEMORYDEVICE_CPU)[i].y;
                    image3[i * 3 + 2] = itmImage->GetData(MEMORYDEVICE_CPU)[i].z;
                }
                pangolin::GlTexture imageTexture3(itmImage->noDims.x, itmImage->noDims.y, GL_RGB, false, 0, GL_RGB,
                                                  GL_UNSIGNED_BYTE);

                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                imageTexture3.Upload(image3, GL_RGB, GL_UNSIGNED_BYTE);

                itmImage = mainEngine->getImage(3);
                unsigned char *image4 = new unsigned char[itmImage->noDims.x * itmImage->noDims.y * 3];

                noDims = itmImage->noDims;

                for (int i = 0; i < noDims.x * noDims.y; ++i) {
                    image4[i * 3 + 0] = itmImage->GetData(MEMORYDEVICE_CPU)[i].x;
                    image4[i * 3 + 1] = itmImage->GetData(MEMORYDEVICE_CPU)[i].y;
                    image4[i * 3 + 2] = itmImage->GetData(MEMORYDEVICE_CPU)[i].z;
                }
                pangolin::GlTexture imageTexture4(itmImage->noDims.x, itmImage->noDims.y, GL_RGB, false, 0, GL_RGB,
                                                  GL_UNSIGNED_BYTE);

                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                imageTexture4.Upload(image4, GL_RGB, GL_UNSIGNED_BYTE);

                //display the image
                d_image.Activate();
                glColor3f(1.0, 1.0, 1.0);
                imageTexture.RenderToViewport(true); //flip = true

                //display the image
                d_image2.Activate();
                glColor3f(1.0, 1.0, 1.0);
                imageTexture2.RenderToViewport(true);

                //display the image
                d_image3.Activate();
                glColor3f(1.0, 1.0, 1.0);
                imageTexture3.RenderToViewport(true);

                //display the image
                d_image4.Activate();
                glColor3f(1.0, 1.0, 1.0);
                imageTexture4.RenderToViewport(true);

                delete[] image;
                delete[] image2;
                delete[] image3;
                delete[] image4;*/




            }


        /*
        const int width =  64;
        const int height = 48;

        unsigned char* imageArray = new unsigned char[3*width*height];
        pangolin::GlTexture imageTexture(width,height,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

        // Default hooks for exiting (Esc) and fullscreen (tab).
        while(!pangolin::ShouldQuit())
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            d_cam.Activate(s_cam);

            glColor3f(1.0,1.0,1.0);
            pangolin::glDrawColouredCube();

            //Set some random image data and upload to GPU
            setImageData(imageArray,3*width*height);
            imageTexture.Upload(imageArray,GL_RGB,GL_UNSIGNED_BYTE);

            //display the image
            d_image.Activate();
            glColor3f(1.0,1.0,1.0);
            imageTexture.RenderToViewport();

            pangolin::FinishFrame();
        }

        delete[] imageArray;*/


    }

}