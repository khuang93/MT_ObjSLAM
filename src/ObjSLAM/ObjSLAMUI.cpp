//
// Created by khuang on 9/24/18.
//

#include "ObjSLAMUI.h"
#include <thread>

namespace ObjSLAM {

    void ObjSLAMUI::ProcessFrame() {
        if (imgNum <= totFrames) {
            sceneIsBackground = true;
            imgNum = mainEngine->ReadNext();
            mainEngine->TrackFrame();
            mainEngine->UpdateMappingEngine();
            mainEngine->MapFrame();

            if(imgNum%10==0) mainEngine->OutputPics();

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

    void ObjSLAMUI::CreateObjectsDisplay() {
        // Create OpenGL window in single line
        pangolin::CreateWindowAndBind("ObjSLAM Objects", w, h);

    }


    void ObjSLAMUI::SelectNextObj() {
        cout << "Object Number: \n";
        currentObjNum = (++currentObjNum) % this->mainEngine->GetActiveObjNumber();
        cout << currentObjNum << endl;
    }

    void ObjSLAMUI::SelectPrevObj() {
        cout << "Object Number: \n";
        currentObjNum =
                (--currentObjNum + this->mainEngine->GetActiveObjNumber()) % this->mainEngine->GetActiveObjNumber();
        cout << currentObjNum << endl;
    }

    void ObjSLAMUI::Pause() {
        cout << "Pause ";
        continueProcess = !continueProcess;
        cout << !continueProcess << endl;
    }

    void ObjSLAMUI::OneFrame() {
        Pause();
        ProcessFrame();
    }

    void ObjSLAMUI::Continuous() {
        cout << "Continuous ";
        continueProcess = true;
        cout << continueProcess << endl;
        ProcessContinuous();
    }


    void ObjSLAMUI::Reg() {
        pangolin::RegisterKeyPressCallback(pangolin::PANGO_KEY_RIGHT, [this]() { SelectNextObj(); });
        pangolin::RegisterKeyPressCallback(pangolin::PANGO_KEY_LEFT, [this]() { SelectPrevObj(); });

        pangolin::RegisterKeyPressCallback('>', [this]() { SelectNextObj(); });
        pangolin::RegisterKeyPressCallback('<', [this]() { SelectPrevObj(); });

        pangolin::RegisterKeyPressCallback('.', [this]() { SelectNextObj(); });
        pangolin::RegisterKeyPressCallback(',', [this]() { SelectPrevObj(); });

        pangolin::RegisterKeyPressCallback('p', [this]() { Pause(); });
        pangolin::RegisterKeyPressCallback('c', [this]() { Continuous(); });
        pangolin::RegisterKeyPressCallback('n', [this]() { OneFrame(); });

    }

    void ObjSLAMUI::Run() {

        CreateDisplay();

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);


        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000),
                pangolin::ModelViewLookAt(-1, 1, -1, 0, 0, 0, pangolin::AxisY)
        );


        Reg();

        pangolin::View &d_image_BG = pangolin::Display("image")
                .SetAspect(640.0f / 480.0f);



        pangolin::View &d_image_rgb = pangolin::Display("image_rgb")
                .SetAspect(640.0f / 480.0f);


        pangolin::View &d_image_above = pangolin::Display("image_above")
                .SetAspect(640.0f / 480.0f);

        pangolin::View &d_image_obj = pangolin::Display("d_image_obj")
                .SetAspect(640.0f / 480.0f);
        pangolin::View &d_image_obj2 = pangolin::Display("d_image_obj2")
                .SetAspect(640.0f / 480.0f);
        pangolin::View &d_image_obj3 = pangolin::Display("d_image_obj3")
                .SetAspect(640.0f / 480.0f);
        pangolin::View &d_image_obj4 = pangolin::Display("d_image_obj4")
                .SetAspect(640.0f / 480.0f);
        pangolin::View &d_image_obj5 = pangolin::Display("d_image_obj5")
                .SetAspect(640.0f / 480.0f);
        pangolin::View &d_image_obj6 = pangolin::Display("d_image_obj6")
                .SetAspect(640.0f / 480.0f);


        pangolin::Display("multi")
                .SetBounds(0.0, 1.0, 0.0, 1.0)
                .SetLayout(pangolin::LayoutEqual)
                .AddDisplay(d_image_BG)
                .AddDisplay(d_image_rgb)
                .AddDisplay(d_image_above)
                .AddDisplay(d_image_obj)
                .AddDisplay(d_image_obj2)
                .AddDisplay(d_image_obj3)
                .AddDisplay(d_image_obj4)
                .AddDisplay(d_image_obj5)
                .AddDisplay(d_image_obj6);


        std::cout << "Resize the window to experiment with SetBounds, SetLock and SetAspect." << std::endl;

        ProcessFrame();
        while (imgNum != -1 && !pangolin::ShouldQuit()) {


            auto *itmImage_BG = mainEngine->GetBGImage();
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


            auto *itmImage_obj = mainEngine->GetImage(currentObjNum);
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


            auto *itmImage_rgb = mainEngine->GetInputImage();

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
            auto *itmImage_above = mainEngine->GetAboveImage();

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

            //

            auto *itmImage_obj2 = mainEngine->GetImage(currentObjNum+1);
            unsigned char *image_obj2 = new unsigned char[itmImage_obj2->noDims.x * itmImage_obj2->noDims.y * 3];


            for (int i = 0; i < noDims.x * noDims.y; ++i) {
                image_obj2[i * 3 + 0] = itmImage_obj2->GetData(MEMORYDEVICE_CPU)[i].x;
                image_obj2[i * 3 + 1] = itmImage_obj2->GetData(MEMORYDEVICE_CPU)[i].y;
                image_obj2[i * 3 + 2] = itmImage_obj2->GetData(MEMORYDEVICE_CPU)[i].z;
            }
            pangolin::GlTexture imageTexture_obj2(itmImage_obj2->noDims.x, itmImage_obj2->noDims.y, GL_RGB, false, 0,
                                                 GL_RGB,
                                                 GL_UNSIGNED_BYTE);

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            imageTexture_obj2.Upload(image_obj2, GL_RGB, GL_UNSIGNED_BYTE);

            //
            auto *itmImage_obj3 = mainEngine->GetImage(currentObjNum+2);
            unsigned char *image_obj3 = new unsigned char[itmImage_obj3->noDims.x * itmImage_obj3->noDims.y * 3];


            for (int i = 0; i < noDims.x * noDims.y; ++i) {
                image_obj3[i * 3 + 0] = itmImage_obj3->GetData(MEMORYDEVICE_CPU)[i].x;
                image_obj3[i * 3 + 1] = itmImage_obj3->GetData(MEMORYDEVICE_CPU)[i].y;
                image_obj3[i * 3 + 2] = itmImage_obj3->GetData(MEMORYDEVICE_CPU)[i].z;
            }
            pangolin::GlTexture imageTexture_obj3(itmImage_obj3->noDims.x, itmImage_obj3->noDims.y, GL_RGB, false, 0,
                                                  GL_RGB,
                                                  GL_UNSIGNED_BYTE);

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            imageTexture_obj3.Upload(image_obj3, GL_RGB, GL_UNSIGNED_BYTE);

            //
            auto *itmImage_obj4 = mainEngine->GetImage(currentObjNum+3);
            unsigned char *image_obj4 = new unsigned char[itmImage_obj4->noDims.x * itmImage_obj4->noDims.y * 3];


            for (int i = 0; i < noDims.x * noDims.y; ++i) {
                image_obj4[i * 3 + 0] = itmImage_obj4->GetData(MEMORYDEVICE_CPU)[i].x;
                image_obj4[i * 3 + 1] = itmImage_obj4->GetData(MEMORYDEVICE_CPU)[i].y;
                image_obj4[i * 3 + 2] = itmImage_obj4->GetData(MEMORYDEVICE_CPU)[i].z;
            }
            pangolin::GlTexture imageTexture_obj4(itmImage_obj4->noDims.x, itmImage_obj4->noDims.y, GL_RGB, false, 0,
                                                  GL_RGB,
                                                  GL_UNSIGNED_BYTE);

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            imageTexture_obj4.Upload(image_obj4, GL_RGB, GL_UNSIGNED_BYTE);


            //
            auto *itmImage_obj5 = mainEngine->GetImage(currentObjNum+4);
            unsigned char *image_obj5 = new unsigned char[itmImage_obj5->noDims.x * itmImage_obj5->noDims.y * 3];


            for (int i = 0; i < noDims.x * noDims.y; ++i) {
                image_obj5[i * 3 + 0] = itmImage_obj5->GetData(MEMORYDEVICE_CPU)[i].x;
                image_obj5[i * 3 + 1] = itmImage_obj5->GetData(MEMORYDEVICE_CPU)[i].y;
                image_obj5[i * 3 + 2] = itmImage_obj5->GetData(MEMORYDEVICE_CPU)[i].z;
            }
            pangolin::GlTexture imageTexture_obj5(itmImage_obj5->noDims.x, itmImage_obj5->noDims.y, GL_RGB, false, 0,
                                                  GL_RGB,
                                                  GL_UNSIGNED_BYTE);

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            imageTexture_obj5.Upload(image_obj5, GL_RGB, GL_UNSIGNED_BYTE);

            //
            auto *itmImage_obj6 = mainEngine->GetImage(currentObjNum+4);
            unsigned char *image_obj6 = new unsigned char[itmImage_obj6->noDims.x * itmImage_obj6->noDims.y * 3];


            for (int i = 0; i < noDims.x * noDims.y; ++i) {
                image_obj6[i * 3 + 0] = itmImage_obj6->GetData(MEMORYDEVICE_CPU)[i].x;
                image_obj6[i * 3 + 1] = itmImage_obj6->GetData(MEMORYDEVICE_CPU)[i].y;
                image_obj6[i * 3 + 2] = itmImage_obj6->GetData(MEMORYDEVICE_CPU)[i].z;
            }
            pangolin::GlTexture imageTexture_obj6(itmImage_obj6->noDims.x, itmImage_obj6->noDims.y, GL_RGB, false, 0,
                                                  GL_RGB,
                                                  GL_UNSIGNED_BYTE);

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            imageTexture_obj6.Upload(image_obj6, GL_RGB, GL_UNSIGNED_BYTE);





            d_image_BG.Activate();
            glColor3f(1.0, 1.0, 1.0);
            imageTexture_BG.RenderToViewport(true);

            d_image_rgb.Activate();
            glColor3f(1.0, 1.0, 1.0);
            imageTexture_rgb.RenderToViewport(true);

            d_image_above.Activate();
            glColor3f(1.0, 1.0, 1.0);
            imageTexture_above.RenderToViewport(true);

            d_image_obj.Activate();
            glColor3f(1.0, 1.0, 1.0);
            imageTexture_obj.RenderToViewport(true);

            d_image_obj2.Activate();
            glColor3f(1.0, 1.0, 1.0);
            imageTexture_obj2.RenderToViewport(true);

            d_image_obj3.Activate();
            glColor3f(1.0, 1.0, 1.0);
            imageTexture_obj3.RenderToViewport(true);

            d_image_obj4.Activate();
            glColor3f(1.0, 1.0, 1.0);
            imageTexture_obj4.RenderToViewport(true);

            d_image_obj5.Activate();
            glColor3f(1.0, 1.0, 1.0);
            imageTexture_obj5.RenderToViewport(true);

            d_image_obj6.Activate();
            glColor3f(1.0, 1.0, 1.0);
            imageTexture_obj6.RenderToViewport(true);


            pangolin::FinishFrame();

            delete[] image_BG;
            delete[] image_obj;
            delete[] image_obj2;
            delete[] image_obj3;
            delete[] image_obj4;
            delete[] image_obj5;

            delete[] image_rgb;
            delete[] image_above;


            ProcessContinuous();

//            ProcessFrame();
        }

    }


}