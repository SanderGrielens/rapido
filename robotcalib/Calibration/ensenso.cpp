#include "calibration.hpp"
#include "calib_en/nxLib.h"

void get_en_image()
{
    char flag = 'g';
    int i = 0;
    while(flag != 'q')
    {
        ostringstream conv;
        conv << i;
        cout<<"Capturing new calibration image from the ensenso stereo vision camera."<<endl;
        ///Read the Ensenso stereo cameras:
        try {
            // Initialize NxLib and enumerate cameras
            nxLibInitialize(true);

            // Reference to the first camera in the node BySerialNo
            NxLibItem root;
            NxLibItem camera = root[itmCameras][itmBySerialNo][0];

            // Open the Ensenso
            NxLibCommand open(cmdOpen);
            open.parameters()[itmCameras] = camera[itmSerialNumber].asString();
            open.execute();

            // Capture an image
            NxLibCommand (cmdCapture).execute();

            NxLibCommand (cmdRectifyImages).execute();

            // Save images
            NxLibCommand saveImage(cmdSaveImage);
            //   raw left
            saveImage.parameters()[itmNode] = camera[itmImages][itmRaw][itmLeft].path;
            saveImage.parameters()[itmFilename] = "calib_en/raw_left" + conv.str()+".png";
            saveImage.execute();
            //   raw right
            /*saveImage.parameters()[itmNode] = camera[itmImages][itmRaw][itmRight].path;
            saveImage.parameters()[itmFilename] = "calib_en/raw_right.png";
            saveImage.execute();
            //   rectified left
            saveImage.parameters()[itmNode] = camera[itmImages][itmRectified][itmLeft].path;
            saveImage.parameters()[itmFilename] = "calib_en/rectified_left.png";
            saveImage.execute();
            //   rectified right
            saveImage.parameters()[itmNode] = camera[itmImages][itmRectified][itmRight].path;
            saveImage.parameters()[itmFilename] = "calib_en/rectified_right.png";
            saveImage.execute();*/
        } catch (NxLibException& e) { // Display NxLib API exceptions, if any
            printf("An NxLib API error with code %d (%s) occurred while accessing item %s.\n", e.getErrorCode(), e.getErrorText().c_str(), e.getItemPath().c_str());
            if (e.getErrorCode() == NxLibExecutionFailed) printf("/Execute:\n%s\n", NxLibItem(itmExecute).asJson(true).c_str());
        } catch (...) { // Display other exceptions
            printf("Something, somewhere went terribly wrong!\n");
        }

        /*cout<<"Plug in the RGB camera and press any key to continue."<<endl;
        cin.ignore();
        cin.get();*/
        cout<<"Capturing new calibration image from the ensenso RGB camera."<<endl;

        ///Read the IDS RGB Camera attached to the Ensenso stereo camera
        HIDS hCam = 0;
        printf("Success-Code: %d\n",IS_SUCCESS);
        //Kamera öffnen
        INT nRet = is_InitCamera (&hCam, NULL);
        printf("Status Init %d\n",nRet);

        //Pixel-Clock setzen
        UINT nPixelClockDefault = 9;
        nRet = is_PixelClock(hCam, IS_PIXELCLOCK_CMD_SET,
                            (void*)&nPixelClockDefault,
                            sizeof(nPixelClockDefault));

        printf("Status is_PixelClock %d\n",nRet);

        //Farbmodus der Kamera setzen
        //INT colorMode = IS_CM_CBYCRY_PACKED;
        INT colorMode = IS_CM_BGR8_PACKED;

        nRet = is_SetColorMode(hCam,colorMode);
        printf("Status SetColorMode %d\n",nRet);

        UINT formatID = 4;
        //Bildgröße einstellen -> 2592x1944
        nRet = is_ImageFormat(hCam, IMGFRMT_CMD_SET_FORMAT, &formatID, 4);
        printf("Status ImageFormat %d\n",nRet);

        //Speicher für Bild alloziieren
        char* pMem = NULL;
        int memID = 0;
        nRet = is_AllocImageMem(hCam, 1280, 1024, 24, &pMem, &memID);
        printf("Status AllocImage %d\n",nRet);

        //diesen Speicher aktiv setzen
        nRet = is_SetImageMem(hCam, pMem, memID);
        printf("Status SetImageMem %d\n",nRet);

        //Bilder im Kameraspeicher belassen
        INT displayMode = IS_SET_DM_DIB;
        nRet = is_SetDisplayMode (hCam, displayMode);
        printf("Status displayMode %d\n",nRet);

        //Bild aufnehmen
        nRet = is_FreezeVideo(hCam, IS_WAIT);
        printf("Status is_FreezeVideo %d\n",nRet);

        //Bild aus dem Speicher auslesen und als Datei speichern
        String path = "./calib_en/snap_BGR"+conv.str()+".png";
        std::wstring widepath;
        for(int i = 0; i < path.length(); ++i)
          widepath += wchar_t (path[i] );

        IMAGE_FILE_PARAMS ImageFileParams;
        ImageFileParams.pwchFileName = &widepath[0];
        ImageFileParams.pnImageID = NULL;
        ImageFileParams.ppcImageMem = NULL;
        ImageFileParams.nQuality = 0;
        ImageFileParams.nFileType = IS_IMG_PNG;

        nRet = is_ImageFile(hCam, IS_IMAGE_FILE_CMD_SAVE, (void*) &ImageFileParams, sizeof(ImageFileParams));
        printf("Status is_ImageFile %d\n",nRet);

        /*ImageFileParams.pwchFileName = L"./calib_en/snap_BGR8.bmp";
        ImageFileParams.pnImageID = NULL;
        ImageFileParams.ppcImageMem = NULL;
        ImageFileParams.nQuality = 0;
        ImageFileParams.nFileType = IS_IMG_BMP;

        nRet = is_ImageFile(hCam, IS_IMAGE_FILE_CMD_SAVE, (void*) &ImageFileParams, sizeof(ImageFileParams));
        printf("Status is_ImageFile %d\n",nRet);

        ImageFileParams.pwchFileName = L"./calib_en/snap_BGR8.jpg";
        ImageFileParams.pnImageID = NULL;
        ImageFileParams.ppcImageMem = NULL;
        ImageFileParams.nQuality = 0;
        ImageFileParams.nFileType = IS_IMG_JPG;

        nRet = is_ImageFile(hCam, IS_IMAGE_FILE_CMD_SAVE, (void*) &ImageFileParams, sizeof(ImageFileParams));
        printf("Status is_ImageFile %d\n",nRet);*/

        //Kamera wieder freigeben
        is_ExitCamera(hCam);
        cout<<"To quit capturing calibration images, choose q. Else, choose any other letter."<<endl;
        cin >> flag;
        i++;
    }

	/*VideoCapture cap(CV_CAP_PVAPI);
    if(!cap.isOpened())  // check if we succeeded
        cout<<"Not working"<<endl;

    Mat frame;
    cap >> frame;
    */
    //tonen(frame,"kijk");
}

