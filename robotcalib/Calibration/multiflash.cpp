#include "calibration.hpp"

HIDS initCam()
{
    HIDS hCam = 0;
    printf("Success-Code: %d\n",IS_SUCCESS);
    //Kamera öffnen
    INT nRet = is_InitCamera (&hCam, NULL);
    printf("Status Init %d\n",nRet);

    /*//Pixel-Clock setzen
    UINT nPixelClockDefault = 9;
    nRet = is_PixelClock(hCam, IS_PIXELCLOCK_CMD_SET,
                        (void*)&nPixelClockDefault,
                        sizeof(nPixelClockDefault));

    printf("Status is_PixelClock %d\n",nRet);
*/
    //Farbmodus der Kamera setzen
    //INT colorMode = IS_CM_CBYCRY_PACKED;
    INT colorMode = IS_CM_BGR8_PACKED;

    nRet = is_SetColorMode(hCam,IS_CM_SENSOR_RAW8);
    printf("Status SetColorMode %d\n",nRet);

    /*UINT formatID = 4;
    //Bildgröße einstellen -> 2592x1944
    nRet = is_ImageFormat(hCam, IMGFRMT_CMD_SET_FORMAT, &formatID, 5);
    printf("Status ImageFormat %d\n",nRet);
    */
    //Speicher für Bild alloziieren
    char* pMem = NULL;
    int memID = 0;
    nRet = is_AllocImageMem(hCam, 768, 576, 8, &pMem, &memID);
    printf("Status AllocImage %d\n",nRet);

    //diesen Speicher aktiv setzen
    nRet = is_SetImageMem(hCam, pMem, memID);
    printf("Status SetImageMem %d\n",nRet);

    //Bilder im Kameraspeicher belassen
    INT displayMode = IS_SET_DM_DIB;
    nRet = is_SetDisplayMode (hCam, displayMode);
    printf("Status displayMode %d\n",nRet);

    nRet = is_SetExternalTrigger(hCam, IS_SET_TRIGGER_SOFTWARE); ///Trigger camera from software

    return hCam;
}

Mat grabSingleImage(String a, HIDS hCam, bool write)
{
    ///Automatically saves the images as .png

    INT nRet;
    //Bild aufnehmen
    nRet = is_FreezeVideo(hCam, 100);
    printf("Status is_FreezeVideo %d\n",nRet);



    Mat res (576,768, CV_8UC1);
    VOID* pMem_b;
    int retInt = is_GetImageMem(hCam, &pMem_b);
    if (retInt != IS_SUCCESS)
    {
        cout << "Image data could not be read from memory!" << endl;
    }

    memcpy(res.ptr(), pMem_b, res.cols * res.rows);

    if(write)
    {
        String name = "calib_mf/"+a+".png";
        imwrite(name, res);
    }


    if(res.empty())
        cout<<"res is empty"<<endl;
    sleep(1);
    return res;
}

void grabMultiflashImages()
{
    asio::io_service io;
	asio::serial_port port(io);

	port.open("/dev/ttyACM0");
	port.set_option(asio::serial_port_base::baud_rate(9600));

	// Read 1 character into c, this will block
	// forever if no character arrives.



    HIDS hCam = initCam();
    unsigned char command[1] = {0};

    ///Light left
    command[0] = static_cast<unsigned char>( 'c' );
    asio::write(port,boost::asio::buffer(command,1));
    ///Grab image
    Mat left = grabSingleImage("left", hCam, true);
    //tonen(left,"left");

    ///Light right
    command[0] = static_cast<unsigned char>( 'b' );
    asio::write(port,boost::asio::buffer(command,1));    ///Grab image
    Mat right = grabSingleImage("right", hCam, true);
    //tonen(right,"right");

    ///Light up
    command[0] = static_cast<unsigned char>( 'd' );
    asio::write(port,boost::asio::buffer(command,1));
    ///Grab image
    Mat up = grabSingleImage("up", hCam, true);
    //tonen(up, "up");

    ///Light down
    command[0] = static_cast<unsigned char>( 'a' );
    asio::write(port,boost::asio::buffer(command,1));
    ///Grab image
    Mat down = grabSingleImage("down", hCam,true);
    //tonen(down, "down");

	port.close();

    //Kamera wieder freigeben
    is_ExitCamera(hCam);


}

void grabMultiflashCalibImages(int number)
{
    HIDS hCam = initCam();

    for(int i=0; i<number; i++)
    {
        ostringstream conv;
        conv << i;
        ///grab image from camera + save it
        Mat calib_image = grabSingleImage("calib" + conv.str(), hCam, true);
        ///show image and wait for input. this gives user time to reposition calibration board
        if(!calib_image.empty())
            tonen(calib_image, "Image"+conv.str());
        else
        {
            cout<<"Couldn't find calibration image, reposition the board and press enter to continue."<<endl;
            i--;
            cin.ignore();
        }
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

void calculateEdgeMap()
{
    ///Read the images acquired by grabMultiflashImages()
    Mat left, right, down, up;

    string start = "./calib_mf/";
    string links = "left.png";
    string rechts = "right.png";
    string beneden = "down.png";
    string omhoog = "up.png";

    int aantal_sets = 6;

    for(int i = 0; i< aantal_sets; i++)
    {
        ostringstream nummer;
        nummer << i;
        left = imread(start + nummer.str() + links, 0);
        right = imread(start + nummer.str() + rechts, 0);
        down = imread(start + nummer.str() + beneden, 0);
        up = imread(start + nummer.str() + omhoog, 0);

        /*left.clone().convertTo(left, CV_32FC1);
        right.clone().convertTo(right, CV_32FC1);
        down.clone().convertTo(down, CV_32FC1);
        up.clone().convertTo(up, CV_32FC1);
    */

        normalize(left.clone(), left, 0, 255, NORM_MINMAX);
        normalize(right.clone(), right, 0, 255, NORM_MINMAX);
        normalize(down.clone(), down, 0, 255, NORM_MINMAX);
        normalize(up.clone(), up, 0, 255, NORM_MINMAX);

        /*tonen(left, "ratio_left");
        tonen(right, "ratio_right");
        tonen(up, "ratio_up");
        tonen(down, "ratio_down");
*/
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

        ///Calculate shadow free image
        //Mat noshadow = Mat(left.rows, left.cols, CV_32FC1);
        Mat noshadow = Mat(left.rows, left.cols, CV_8UC1);

        for(int y=0; y<left.rows; y++)
        {
            for(int x=0; x<left.cols; x++)
            {
                    uchar color = 0;
                    uchar color_left = left.at<uchar>(y,x);
                    uchar color_right = right.at<uchar>(y,x);
                    uchar color_up = up.at<uchar>(y,x);
                    uchar color_down = down.at<uchar>(y,x);

                    color = ((color < color_left) ? color_left : color);
                    color = ((color < color_right) ? color_right : color);
                    color = ((color < color_up) ? color_up : color);
                    color = ((color < color_down) ? color_down : color);

                    //cout<<color_left<<" "<<color_right<<" "<<color_up<<" "<<color_down<<" "<<color<<endl;

                    noshadow.at<uchar>(y,x) = color;
            }
        }

        ///Calculate difference image
        Mat ratio_left = Mat(left.rows, left.cols, CV_32FC1);
        Mat ratio_right = Mat(left.rows, left.cols, CV_32FC1);
        Mat ratio_down = Mat(left.rows, left.cols, CV_32FC1);
        Mat ratio_up = Mat(left.rows, left.cols, CV_32FC1);

        divide(left, noshadow, ratio_left);
        divide(right, noshadow, ratio_right);
        divide(up, noshadow, ratio_up);
        divide(down, noshadow, ratio_down);

        ratio_left *= 255;
        ratio_right *= 255;
        ratio_down *= 255;
        ratio_up *= 255;

        ratio_left.clone().convertTo(ratio_left, CV_8UC1);
        ratio_right.clone().convertTo(ratio_right, CV_8UC1);
        ratio_down.clone().convertTo(ratio_down, CV_8UC1);
        ratio_up.clone().convertTo(ratio_up, CV_8UC1);

        /*int erosion_type =
                            MORPH_RECT;
                            //MORPH_CROSS;
                            //MORPH_ELLIPSE;
        int erosion = 1;
        Mat element = getStructuringElement( erosion_type,
                                               Size( 2*erosion + 1, 2*erosion+1 ),
                                               Point( erosion, erosion ) );

        dilate(ratio_left.clone(), ratio_left, element );
        dilate(ratio_right.clone(), ratio_right,element );
        dilate(ratio_down.clone(), ratio_down, element );
        dilate(ratio_up.clone(), ratio_up, element );

        erode(ratio_left.clone(), ratio_left, element );
        erode(ratio_right.clone(), ratio_right,element );
        erode(ratio_down.clone(), ratio_down, element );
        erode(ratio_up.clone(), ratio_up, element );*/


        tonen(ratio_left, "ratio_left");
        tonen(ratio_right, "ratio_right");
        tonen(ratio_up, "ratio_up");
        tonen(ratio_down, "ratio_down");
        ///MAYBE SPLIT UP THE SOBEL
        Mat leftkernel = Mat::zeros(3,3, CV_32FC1);
        Mat upkernel = Mat::zeros(3,3, CV_32FC1);

        leftkernel.at<float>(0,0) =1 ;
        leftkernel.at<float>(0,1) =0 ;
        leftkernel.at<float>(0,2) =-1 ;
        leftkernel.at<float>(1,0) =2 ;
        leftkernel.at<float>(1,1) =0 ;
        leftkernel.at<float>(1,2) =-2 ;
        leftkernel.at<float>(2,0) =1 ;
        leftkernel.at<float>(2,1) =0 ;
        leftkernel.at<float>(2,2) =-1 ;

        upkernel.at<float>(0,0) =1 ;
        upkernel.at<float>(0,1) =2 ;
        upkernel.at<float>(0,2) =1 ;
        upkernel.at<float>(1,0) =0 ;
        upkernel.at<float>(1,1) =0 ;
        upkernel.at<float>(1,2) =0 ;
        upkernel.at<float>(2,0) =-1 ;
        upkernel.at<float>(2,1) =-2 ;
        upkernel.at<float>(2,2) =-1 ;

        //Sobel( ratio_left.clone(), ratio_left, CV_8UC1, 1, 0, 3 ); //Kernel in de verkeerde richting --> [(1 0 -1), ( 2 0 -2), (1 0 -1)
        filter2D(ratio_left.clone(), ratio_left, CV_8UC1, leftkernel);
        Sobel( ratio_right.clone(), ratio_right, CV_8UC1, 1, 0, 3 );
        Sobel( ratio_down.clone(), ratio_down, CV_8UC1, 0, 1, 3 );
        //Sobel( ratio_up.clone(), ratio_up, CV_8UC1, 0, 1, 3 ); //Kernel in de verkeerde richting --> [(1 2 1), ( 0 0 0), (-1 -2 -1)]
        filter2D(ratio_up.clone(), ratio_up, CV_8UC1, upkernel);


        convertScaleAbs( ratio_left.clone(), ratio_left );
        convertScaleAbs( ratio_right.clone(), ratio_right);
        convertScaleAbs( ratio_down.clone(), ratio_down );
        convertScaleAbs( ratio_up.clone(), ratio_up );


        ///Calculate depth edges
        //tonen(noshadow, "noshadow");
        /*tonen(ratio_left, "sobel_left");
        tonen(ratio_right, "sobel_right");
        tonen(ratio_up, "sobel_up");
        tonen(ratio_down, "sobel_down");*/

        Mat weigthed;
        addWeighted(ratio_left, 1, ratio_right, 1, 0, weigthed);
        addWeighted(weigthed, 1, ratio_down, 1, 0, weigthed);
        addWeighted(weigthed, 1, ratio_up, 1, 0, weigthed);

        Mat edge = Mat::ones(left.rows, left.cols, CV_8UC1);

        Canny(weigthed, edge, 0.1, 0.9, 3 );
        tonen(edge, "result");


        /*edge *=255;

        int grens = 100 ;

        ///Right:
        for(int y=0; y<ratio_right.rows; y++)
        {
            for(int x=ratio_right.cols-1; x>=0; x--)
            {
                if(edge.at<uchar>(y,x) == 255 )
                {
                    if(ratio_right.at<uchar>(y,x+1) - ratio_right.at<uchar>(y,x) > grens  )
                    {
                        edge.at<uchar>(y,x) = 0;
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
                    if(ratio_left.at<uchar>(y,x-1) - ratio_left.at<uchar>(y,x) > grens )
                    {
                        edge.at<uchar>(y,x) = 0;
                    }
                }
            }
        }


        ///Up:
        for(int x=0; x<ratio_up.cols; x++)
        {
            for(int y=0; y<ratio_up.rows; y++)
            {
                if(edge.at<uchar>(y,x) == 255 )
                {
                    if(ratio_up.at<uchar>(y-1,x) - ratio_up.at<uchar>(y,x) > grens  )
                    {
                        edge.at<uchar>(y,x) = 0;
                    }
                }
            }
        }

        ///Down:
        for(int x=0; x<ratio_down.cols; x++)
        {
            for(int y=ratio_down.rows-1; y>=0; y--)
            {
                if(edge.at<uchar>(y,x) == 255 )
                {
                    if(ratio_down.at<uchar>(y+1,x) - ratio_down.at<uchar>(y,x) > grens)
                    {
                        edge.at<uchar>(y,x) = 0;
                    }
                }
            }
        }*/
        tonen(edge, "edges"+nummer.str());
        //imwrite("./calib_mf/"+nummer.str() +"edges.jpg", edge);
    }
}


void calculateEdgeMapRGB()
{
    ///Read the images acquired by grabMultiflashImages()
    Mat left, right, down, up;
    left = imread("./calib_mf/left.png", 1);
    right = imread("./calib_mf/right.png", 1);
    down = imread("./calib_mf/down.png", 1);
    up = imread("./calib_mf/up.png", 1);

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
