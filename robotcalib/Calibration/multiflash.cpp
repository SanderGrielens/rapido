#include "calibration.hpp"

void grabMultiflashImages()
{

}

void grabMultiflashCalibImages(int number)
{
    for(int i=0; i<number; i++)
    {
        ///grab image from camera
        ///show image and wait for input. this gives user time to reposition calibration board
        ///save image
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

        string pad = "./calib_mf/calib" + conv.str()+".bmp";
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
    Mat cameramatrix;

    fs["cameramatrix"] >> cameramatrix;
    */
    ///Calculate shadow free image
    Mat noshadow = Mat(left.rows, left.cols, CV_8UC3);
    Mat ratio_left = Mat(left.rows, left.cols, CV_32FC3);
    Mat ratio_right = Mat(left.rows, left.cols, CV_32FC3);
    Mat ratio_down = Mat(left.rows, left.cols, CV_32FC3);
    Mat ratio_up = Mat(left.rows, left.cols, CV_32FC3);


    for(int y=0; y<left.rows; y++)
    {
        for(int x=0; x<left.cols; x++)
        {
                Scalar color;
                Scalar color2;
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

                if(x == 715 && y == 320)
                {
                    cout<<"blue:  up: "<<color_up.val[0]<<" down: "<<color_down.val[0]<<" right: "<<color_right.val[0]<<endl;
                    cout<<"green: up: "<<color_up.val[1]<<" down: "<<color_down.val[1]<<" right: "<<color_right.val[1]<<endl;
                    cout<<"red:   up: "<<color_up.val[2]<<" down: "<<color_down.val[2]<<" right: "<<color_right.val[2]<<endl;

                    cout<<"opgeslagen: "<<color<<endl;

                }

                //cout<<"New BGR value: "<<blue<<" "<<green<<" "<<red<<endl;

                noshadow.at<Vec3b>(y,x)[0] = color.val[0];
                noshadow.at<Vec3b>(y,x)[1] = color.val[1];
                noshadow.at<Vec3b>(y,x)[2] = color.val[2];

                ratio_left.at<Vec3f>(y,x)[0] = color_left.val[0]/color.val[0]*255;
                ratio_left.at<Vec3f>(y,x)[1] = color_left.val[1]/color.val[1]*255;
                ratio_left.at<Vec3f>(y,x)[2] = color_left.val[2]/color.val[2]*255;

                ratio_right.at<Vec3f>(y,x)[0] = color_right.val[0]/color.val[0]*255;
                ratio_right.at<Vec3f>(y,x)[2] = color_right.val[2]/color.val[2]*255;
                ratio_right.at<Vec3f>(y,x)[1] = color_right.val[1]/color.val[1]*255;

                ratio_up.at<Vec3f>(y,x)[0] = color_up.val[0]/color.val[0]*255;
                ratio_up.at<Vec3f>(y,x)[1] = color_up.val[1]/color.val[1]*255;
                ratio_up.at<Vec3f>(y,x)[2] = color_up.val[2]/color.val[2]*255;

                ratio_down.at<Vec3f>(y,x)[0] = color_down.val[0]/color.val[0]*255;
                ratio_down.at<Vec3f>(y,x)[1] = color_down.val[1]/color.val[1]*255;
                ratio_down.at<Vec3f>(y,x)[2] = color_down.val[2]/color.val[2]*255;
        }
    }
    imwrite("noshadow.JPG",noshadow);

    cvtColor(ratio_left, ratio_left, CV_BGR2GRAY);

    tonen(noshadow, "gene schaduw");
    tonen(ratio_left, "left");
    tonen(ratio_right, "right");
    tonen(ratio_up, "up");
    tonen(ratio_down, "down");

    ///Calculate depth edges


}


