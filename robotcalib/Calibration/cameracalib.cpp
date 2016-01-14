#include "calibration.hpp"

void calibrate_camera()
{
    ///Create World Corners:
    vector<cv::Point3f> world_corners;


    for(int h=0; h<6; h++)
    {
        for (int w=0; w<8; w++)
        {
            world_corners.push_back(cv::Point3f(28.55555555f * w, 28.55555555f * h, 0.f));
        }
    }

    string dir_calib_cam = "./calib_cam/";
    vector<string> files_calib_cam;
    int calib_cam_series=0;

    ///Count the number of structured light calibration series
    getdir(dir_calib_cam, files_calib_cam);
    for(uint i=0; i<files_calib_cam.size(); i++)
    {
        calib_cam_series = i;
    }
    ///Remove ".." and "." directoy from the count
    calib_cam_series--;

    Mat board;
    Size boardSize(8,6);
    vector<vector<Point2f> > chessboardcorners(calib_cam_series);
    vector<vector<Point3f> > objectpoints;

    cout<<calib_cam_series<<endl;

    ///Find corners
    for(int i = 1; i<=calib_cam_series; i++)
    {
        cout<<"finding corners image "<<i<<endl;
        ostringstream conv;
        conv << i;

        string pad = "./calib_cam/camera_only_calibration_capture_" + conv.str()+".bmp";
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
            break;
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
}

