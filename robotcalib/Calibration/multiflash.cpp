#include "calibration.hpp"
using namespace LibSerial;


Mat grabSingleImage(String a)
{
    ///Automatically saves the images as .png
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
    String path = "./calib_mf/"+a+".png";
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

    //Kamera wieder freigeben
    is_ExitCamera(hCam);

    Mat res = imread(a, 0);
    return res;
}

void grabMultiflashImages()
{
    SerialStream my_serial_stream;
    my_serial_stream.Open( "/media/sgr/MBED" );
    my_serial_stream.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;

    ///Light left
    my_serial_stream << "a";
    ///Grab image
    Mat left = grabSingleImage("left");
    tonen(left,"left");

    ///Light right
    my_serial_stream << "b";
    ///Grab image
    Mat right = grabSingleImage("right");
    tonen(right,"right");

    ///Light up
    my_serial_stream << "c";
    ///Grab image
    Mat up = grabSingleImage("up");
    tonen(up, "up");
    ///Light down
    my_serial_stream << "d";
    ///Grab image
    Mat down = grabSingleImage("down");
    tonen(down, "down");
    my_serial_stream.Close();

}

void grabMultiflashCalibImages(int number)
{
    for(int i=0; i<number; i++)
    {
        ostringstream conv;
        conv << i;
        ///grab image from camera + save it
        Mat calib_image = grabSingleImage("calib" + conv.str());
        ///show image and wait for input. this gives user time to reposition calibration board
        tonen(calib_image, "Image");
    }
    cout<<"Grab succesful"<<endl;
}

void calibrateMultiflashCamera()
{
    ///Grab calibration images
    cout<<"how many Calibration images?\n";
    int number_calib = 0;
    cin >> number_calib;

    grabMultiflashCalibImages(number_calib);

    ///Create World Corners:
    vector<cv::Point3f> world_corners;

    for(int h=0; h<6; h++)
    {
        for (int w=0; w<8; w++)
        {
            world_corners.push_back(cv::Point3f(27.f * w, 27.f * h, 0.f));
        }
    }

    Mat board;
    Size boardSize(8,6);
    vector<vector<Point2f> > chessboardcorners(number_calib);
    vector<vector<Point3f> > objectpoints;

    ///Find corners
    //#pragma omp parallel for
    for(int i = 0; i<=number_calib; i++)
    {
        cout<<"finding corners image "<<i<<endl;
        ostringstream conv;
        conv << i;

        string pad = "./calib_mf/calib" + conv.str()+".png";
        board = imread(pad, 0);
        if(board.empty())
        {
            cout<<"No image found"<<i<<endl;
            continue;
        }

        bool found1_1 = findChessboardCorners(board, boardSize, chessboardcorners[i-1],CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

        if(!found1_1)
        {
            std::cerr << "Checkboard 1_"<<i<<" corners not found!" << std::endl;
            continue;
        }

        cornerSubPix(board, chessboardcorners[i-1], Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 300, 0.001));
        //cvtColor(board, board, CV_GRAY2BGR);
        //drawChessboardCorners( board, boardSize, chessboardcorners[i-1], found1_1 );
        //tonen(board, "zijn ze gevonden?");
        objectpoints.push_back(world_corners);
    }


    Size imageSize = board.size();

    int cal_flags = 0
                //+ CV_CALIB_FIX_PRINCIPAL_POINT
                //+ cv::CALIB_FIX_K1
                //+ cv::CALIB_FIX_K2
                //+ cv::CALIB_ZERO_TANGENT_DIST
                //+ cv::CALIB_FIX_K3
                ;

    vector<Mat> cam_rvecs, cam_tvecs;
    Mat cam_mat;
    Mat cam_dist;
    int cam_flags = cal_flags;
    double cam_error = cv::calibrateCamera(objectpoints, chessboardcorners, imageSize, cam_mat, cam_dist, cam_rvecs, cam_tvecs, cam_flags,
                                            TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5));
    cout<<"Reprojection error: "<<cam_error<<endl;

    FileStorage fs("./camera.xml", FileStorage::WRITE);
    fs << "reprojection_error" << cam_error;
    fs << "camera_matrix" << cam_mat;
    fs << "distortion_camera" << cam_dist;
    fs.release();
}

void calculateDepthMap()
{
    ///Read the images acquired by grabMultiflashImages()
    Mat left, right, down, up;
    left = imread("./calib_mf/left.JPG", 1);
    right = imread("./calib_mf/right.JPG", 1);
    down = imread("./calib_mf/down.JPG", 1);
    up = imread("./calib_mf/up.JPG", 1);

    if(left.empty())
    {
        cout<<"Left nok"<<endl;
    }

    if(right.empty())
    {
        cout<<"right nok"<<endl;
    }

    if(down.empty())
    {
        cout<<"down nok"<<endl;
    }

    if(up.empty())
    {
        cout<<"up nok"<<endl;
    }

    ///undistort images using cameramatrix calculated in calibrateMultiflashCamera()
    /*FileStorage fs("./camera.xml", FileStorage::READ);
    Mat cameramatrix, distcoef;

    fs["camera_matrix"] >> cameramatrix;
    fs[distortion_camera"] >> distcoef;

    undistort(left, left, cameramatrix, distcoef);
    undistort(right, right, cameramatrix, distcoef);
    undistort(up, up, cameramatrix, distcoef);
    undistort(down, down, cameramatrix, distcoef);

    */
    ///Calculate shadow free image
    Mat noshadow = Mat(left.rows, left.cols, CV_8UC3);

    for(int y=0; y<left.rows; y++)
    {
        for(int x=0; x<left.cols; x++)
        {
                Scalar color;
                Scalar color_left = left.at<Vec3b>(y,x);
                Scalar color_right = right.at<Vec3b>(y,x);
                Scalar color_up = up.at<Vec3b>(y,x);
                Scalar color_down = down.at<Vec3b>(y,x);

                color.val[0] = ((color.val[0] < color_left.val[0]) ? color_left.val[0] : color.val[0]);
                color.val[0] = ((color.val[0] < color_right.val[0]) ? color_right.val[0] : color.val[0]);
                color.val[0] = ((color.val[0] < color_up.val[0]) ? color_up.val[0] : color.val[0]);
                color.val[0] = ((color.val[0] < color_down.val[0]) ? color_down.val[0] : color.val[0]);

                color.val[1] = ((color.val[1] < color_left.val[1]) ? color_left.val[1] : color.val[1]);
                color.val[1] = ((color.val[1] < color_right.val[1]) ? color_right.val[1] : color.val[1]);
                color.val[1] = ((color.val[1] < color_up.val[1]) ? color_up.val[1] : color.val[1]);
                color.val[1] = ((color.val[1] < color_down.val[1]) ? color_down.val[1] : color.val[1]);

                color.val[2] = ((color.val[2] < color_left.val[2]) ? color_left.val[2] : color.val[2]);
                color.val[2] = ((color.val[2] < color_right.val[2]) ? color_right.val[2] : color.val[2]);
                color.val[2] = ((color.val[2] < color_up.val[2]) ? color_up.val[2] : color.val[2]);
                color.val[2] = ((color.val[2] < color_down.val[2]) ? color_down.val[2] : color.val[2]);

                noshadow.at<Vec3b>(y,x)[0] = color.val[0];
                noshadow.at<Vec3b>(y,x)[1] = color.val[1];
                noshadow.at<Vec3b>(y,x)[2] = color.val[2];
        }
    }

    ///Calculate difference image
    Mat ratio_left = Mat(left.rows, left.cols, CV_8UC1);
    Mat ratio_right = Mat(left.rows, left.cols, CV_8UC1);
    Mat ratio_down = Mat(left.rows, left.cols, CV_8UC1);
    Mat ratio_up = Mat(left.rows, left.cols, CV_8UC1);
    Mat left_b, right_b, up_b, down_b, noshadow_b;

    cvtColor(noshadow, noshadow_b, CV_BGR2GRAY);
    cvtColor(left, left_b, CV_BGR2GRAY);
    cvtColor(right, right_b, CV_BGR2GRAY);
    cvtColor(up, up_b, CV_BGR2GRAY);
    cvtColor(down, down_b, CV_BGR2GRAY);

    ratio_left = left_b/noshadow_b;
    ratio_right = right_b/noshadow_b;
    ratio_up = up_b/noshadow_b;
    ratio_down = down_b/noshadow_b;

    ///Calculate depth edges

    Mat edge = Mat::ones(ratio_left.rows, ratio_left.cols, CV_8UC1);
    edge *=255;

    ///Right:
    for(int y=0; y<ratio_right.rows; y++)
    {
        for(int x=ratio_right.cols-1; x>=0; x--)
        {
            if(edge.at<uchar>(y,x) == 255 )
            {
                if(ratio_right.at<uchar>(y,x) < ratio_right.at<uchar>(y,x+1))
                {
                    edge.at<uchar>(y,x) = 0;
                }
                else
                {
                    edge.at<uchar>(y,x) = 255;
                }
            }
        }
    }

    ///Left:
    for(int y=0; y<ratio_left.rows; y++)
    {
        for(int x=0; x<ratio_left.cols; x++)
        {
            if(edge.at<uchar>(y,x) == 255 )
            {
                if(ratio_left.at<uchar>(y,x) < ratio_left.at<uchar>(y,x-1))
                {
                    edge.at<uchar>(y,x) = 0;
                }
                else
                {
                    edge.at<uchar>(y,x) = 255;
                }
            }
        }
    }

    ///Up:
    for(int x=0; x<ratio_up.cols; x++)
    {
        for(int y=0; y<ratio_up.rows; y++)
        {
            if(edge.at<uchar>(y,x) !=0 )
            {
                if(ratio_up.at<uchar>(y,x) < ratio_up.at<uchar>(y-1,x))
                {
                    edge.at<uchar>(y,x) = 0;
                }
                else
                {
                    edge.at<uchar>(y,x) = 255;
                }
            }
        }
    }

    ///Down:
    for(int x=0; x<ratio_down.cols; x++)
    {
        for(int y=ratio_down.rows-1; y>=0; y--)
        {
            if(edge.at<uchar>(y,x) !=0 )
            {
                if(ratio_down.at<uchar>(y,x) < ratio_down.at<uchar>(y+1,x))
                {
                    edge.at<uchar>(y,x) = 0;
                }
                else
                {
                    edge.at<uchar>(y,x) = 255;
                }
            }
        }
    }

    imwrite("edges.jpg", edge);
    tonen(edge, "edges");
}
