//
// Created by khuang on 9/24/18.
//

#include "ObjSLAMUI.h"
#include <thread>

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

    void ObjSLAMUI::ProcessContinuous() {
        if (continueProcess) {
            ProcessFrame();
        }
    }


    void ObjSLAMUI::CreateDisplay() {
        // Create OpenGL window in single line
        pangolin::CreateWindowAndBind("ObjSLAM UI", w, h);
    }


    void ObjSLAMUI::chooseNextObj() {
        cout << "Object Number: \n";
        currentObjNum = (++currentObjNum) % this->mainEngine->getActiveObjNumber();
        cout << currentObjNum << endl;
    }

    void ObjSLAMUI::choosePrevObj() {
        cout << "Object Number: \n";
        currentObjNum =
                (--currentObjNum + this->mainEngine->getActiveObjNumber()) % this->mainEngine->getActiveObjNumber();
        cout << currentObjNum << endl;
    }

    void ObjSLAMUI::pause() {
        cout << "Pause ";
        continueProcess = !continueProcess;
        cout << !continueProcess << endl;
    }

    void ObjSLAMUI::oneFrame() {
        pause();
        ProcessFrame();
    }

    void ObjSLAMUI::continuous() {
        cout << "Continuous ";
        continueProcess = true;
        cout << continueProcess << endl;
        ProcessContinuous();
    }


    void ObjSLAMUI::reg() {
        pangolin::RegisterKeyPressCallback(pangolin::PANGO_KEY_RIGHT, [this]() { chooseNextObj(); });
        pangolin::RegisterKeyPressCallback(pangolin::PANGO_KEY_LEFT, [this]() { choosePrevObj(); });

        pangolin::RegisterKeyPressCallback('>', [this]() { chooseNextObj(); });
        pangolin::RegisterKeyPressCallback('<', [this]() { choosePrevObj(); });

        pangolin::RegisterKeyPressCallback('.', [this]() { chooseNextObj(); });
        pangolin::RegisterKeyPressCallback(',', [this]() { choosePrevObj(); });

        pangolin::RegisterKeyPressCallback('p', [this]() { pause(); });
        pangolin::RegisterKeyPressCallback('c', [this]() { continuous(); });
        pangolin::RegisterKeyPressCallback('n', [this]() { oneFrame(); });

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

/*//        pangolin::View &d_cam = pangolin::Display("cam")
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
//                .SetLock(pangolin::LockLeft, pangolin::LockTop);*/

        pangolin::View &d_image_BG = pangolin::Display("image")
                .SetAspect(640.0f / 480.0f);
        pangolin::View &d_image_obj = pangolin::Display("image2")
                .SetAspect(640.0f / 480.0f);

        pangolin::View &d_image_rgb = pangolin::Display("image_rgb")
                .SetAspect(640.0f / 480.0f);


        pangolin::View &d_image_above = pangolin::Display("image_above")
                .SetAspect(640.0f / 480.0f);


        pangolin::Display("multi")
                .SetBounds(0.0, 1.0, 0.0, 1.0)
                .SetLayout(pangolin::LayoutEqual)
                .AddDisplay(d_image_BG)
                .AddDisplay(d_image_rgb)
                .AddDisplay(d_image_obj)
                .AddDisplay(d_image_above);


        std::cout << "Resize the window to experiment with SetBounds, SetLock and SetAspect." << std::endl;

        ProcessFrame();
        while (imgNum != -1 && !pangolin::ShouldQuit()) {


            auto *itmImage_BG = mainEngine->getBGImage();
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
            pangolin::GlTexture imageTexture_obj(itmImage_obj->noDims.x, itmImage_obj->noDims.y, GL_RGB, false, 0,
                                                 GL_RGB,
                                                 GL_UNSIGNED_BYTE);

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            imageTexture_obj.Upload(image_obj, GL_RGB, GL_UNSIGNED_BYTE);


            auto *itmImage_rgb = mainEngine->getInputImage();

            unsigned char *image_rgb = new unsigned char[itmImage_rgb->noDims.x * itmImage_rgb->noDims.y * 3];


            for (int i = 0; i < noDims.x * noDims.y; ++i) {
                image_rgb[i * 3 + 0] = itmImage_rgb->GetData(MEMORYDEVICE_CPU)[i].x;
                image_rgb[i * 3 + 1] = itmImage_rgb->GetData(MEMORYDEVICE_CPU)[i].y;
                image_rgb[i * 3 + 2] = itmImage_rgb->GetData(MEMORYDEVICE_CPU)[i].z;
            }
            pangolin::GlTexture imageTexture_rgb(itmImage_rgb->noDims.x, itmImage_rgb->noDims.y, GL_RGB, false, 0,
                                                 GL_RGB,
                                                 GL_UNSIGNED_BYTE);

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            imageTexture_rgb.Upload(image_rgb, GL_RGB, GL_UNSIGNED_BYTE);


            //above
            auto *itmImage_above = mainEngine->getAboveImage();

            unsigned char *image_above = new unsigned char[itmImage_above->noDims.x * itmImage_above->noDims.y * 3];


            for (int i = 0; i < noDims.x * noDims.y; ++i) {
                image_above[i * 3 + 0] = itmImage_above->GetData(MEMORYDEVICE_CPU)[i].x;
                image_above[i * 3 + 1] = itmImage_above->GetData(MEMORYDEVICE_CPU)[i].y;
                image_above[i * 3 + 2] = itmImage_above->GetData(MEMORYDEVICE_CPU)[i].z;
            }
            pangolin::GlTexture imageTexture_above(itmImage_above->noDims.x, itmImage_above->noDims.y, GL_RGB, false, 0,
                                                 GL_RGB,
                                                 GL_UNSIGNED_BYTE);

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            imageTexture_above.Upload(image_above, GL_RGB, GL_UNSIGNED_BYTE);




            d_image_BG.Activate();
            glColor3f(1.0, 1.0, 1.0);
            imageTexture_BG.RenderToViewport(true);

            d_image_obj.Activate();
            glColor3f(1.0, 1.0, 1.0);
            imageTexture_obj.RenderToViewport(true);


            d_image_rgb.Activate();
            glColor3f(1.0, 1.0, 1.0);
            imageTexture_rgb.RenderToViewport(true);

            d_image_above.Activate();
            glColor3f(1.0, 1.0, 1.0);
            imageTexture_above.RenderToViewport(true);


            pangolin::FinishFrame();

            delete[] image_BG;
            delete[] image_obj;
            delete[] image_rgb;
            delete[] image_above;


            ProcessContinuous();

//            ProcessFrame();
        }

    }


}