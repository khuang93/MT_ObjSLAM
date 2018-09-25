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


    void ObjSLAMUI::run() {

        CreateDisplay();

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);


        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000),
                pangolin::ModelViewLookAt(-1, 1, -1, 0, 0, 0, pangolin::AxisY)
        );

        // Aspect ratio allows us to constrain width and height whilst fitting within specified
        // bounds. A positive aspect ratio makes a view 'shrink to fit' (introducing empty bars),
        // whilst a negative ratio makes the view 'grow to fit' (cropping the view).
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
        pangolin::View &d_image = pangolin::Display("image")
                .SetAspect(640.0f/480.0f);
        pangolin::View &d_image2 = pangolin::Display("image2")
                .SetAspect(640.0f/480.0f);
        pangolin::View &d_image3 = pangolin::Display("image3")
                .SetAspect(640.0f/480.0f);
        pangolin::View &d_image4 = pangolin::Display("image4")
                .SetAspect(640.0f/480.0f);

        pangolin::Display("multi")
                .SetBounds(0.0, 1.0, 0.0, 1.0)
                .SetLayout(pangolin::LayoutEqual)
                .AddDisplay(d_image)
                .AddDisplay(d_image2)
                .AddDisplay(d_image3)
                .AddDisplay(d_image4);



        std::cout << "Resize the window to experiment with SetBounds, SetLock and SetAspect." << std::endl;
        std::cout << "Notice that the cubes aspect is maintained even though it covers the whole screen." << std::endl;

            while (imgNum <= totFrames && !pangolin::ShouldQuit()) {
                ProcessFrame();
                auto *itmImage = mainEngine->getImage(0);
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
                imageTexture.RenderToViewport();

                //display the image
                d_image2.Activate();
                glColor3f(1.0, 1.0, 1.0);
                imageTexture2.RenderToViewport();

                //display the image
                d_image3.Activate();
                glColor3f(1.0, 1.0, 1.0);
                imageTexture3.RenderToViewport();

                //display the image
                d_image4.Activate();
                glColor3f(1.0, 1.0, 1.0);
                imageTexture4.RenderToViewport();

                delete[] image;
                delete[] image2;
                delete[] image3;
                delete[] image4;

                pangolin::FinishFrame();
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