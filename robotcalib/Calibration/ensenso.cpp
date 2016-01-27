#include "calibration.hpp"
#include "calib_en/nxLib.h"

void ensensoExceptionHandling (const NxLibException &ex,
                          std::string func_nam)
{
  PCL_ERROR ("%s: NxLib error %s (%d) occurred while accessing item %s.\n", func_nam.c_str (), ex.getErrorText ().c_str (), ex.getErrorCode (),
            ex.getItemPath ().c_str ());
  if (ex.getErrorCode () == NxLibExecutionFailed)
  {
    NxLibCommand cmd ("");
    PCL_WARN ("\n%s\n", cmd.result ().asJson (true, 4, false).c_str ());
  }
}


void get_en_image(pcl::PointCloud<pcl::PointXYZ> &cloud)
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

            // Stereo matching task
            NxLibCommand (cmdComputeDisparityMap).execute ();

            // Convert disparity map into XYZ data for each pixel
            NxLibCommand (cmdComputePointMap).execute ();

            // Get info about the computed point map and copy it into a std::vector
            double timestamp;
            std::vector<float> pointMap;
            int width, height;
            camera[itmImages][itmRaw][itmLeft].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);  // Get raw image timestamp
            camera[itmImages][itmPointMap].getBinaryDataInfo (&width, &height, 0, 0, 0, 0);
            camera[itmImages][itmPointMap].getBinaryData (pointMap, 0);

            // Copy point cloud and convert in meters
            //cloud.header.stamp = getPCLStamp (timestamp);
            cloud.resize (height * width);
            cloud.width = width;
            cloud.height = height;
            cloud.is_dense = false;

            // Copy data in point cloud (and convert milimeters in meters)
            for (size_t i = 0; i < pointMap.size (); i += 3)
            {
              cloud.points[i / 3].x = pointMap[i] / 1000.0;
              cloud.points[i / 3].y = pointMap[i + 1] / 1000.0;
              cloud.points[i / 3].z = pointMap[i + 2] / 1000.0;
            }

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
        }
        /*catch (NxLibException &ex)
        {
            ensensoExceptionHandling (ex, "grabSingleCloud");
        }*/
        catch (...) { // Display other exceptions
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
        for(int j = 0; j < path.length(); ++j)
          widepath += wchar_t (path[j] );

        IMAGE_FILE_PARAMS ImageFileParams;
        ImageFileParams.pwchFileName = &widepath[0];
        ImageFileParams.pnImageID = NULL;
        ImageFileParams.ppcImageMem = NULL;
        ImageFileParams.nQuality = 0;
        ImageFileParams.nFileType = IS_IMG_PNG;

        nRet = is_ImageFile(hCam, IS_IMAGE_FILE_CMD_SAVE, (void*) &ImageFileParams, sizeof(ImageFileParams));
        printf("Status is_ImageFile %d\n",nRet);

        //Kamera wieder freigeben
        is_ExitCamera(hCam);
        cout<<"To quit capturing calibration images, choose q. Else, choose any other letter."<<endl;
        cin >> flag;
        i++;
    }
}

pcl::PointCloud<pcl::PointXYZ> get_en_cloud()
{
    ///Doesn't want to create ensensograbber object, so function returns empty cloud
    pcl::EnsensoGrabber::Ptr ensenso_ptr;

    ensenso_ptr.reset (new pcl::EnsensoGrabber);
    ensenso_ptr->openTcpPort ();
    ensenso_ptr->openDevice ();

    pcl::PointCloud<pcl::PointXYZ> cloud;
    ensenso_ptr->grabSingleCloud(cloud);
    ensenso_ptr->closeDevice ();

    vector<int> lijst;
    pcl::removeNaNFromPointCloud(cloud, cloud, lijst );
    return cloud;
}
