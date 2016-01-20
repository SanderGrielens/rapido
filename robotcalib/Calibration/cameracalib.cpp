#include "calibration.hpp"

void calibrate_camera()
{
    ///Create World Corners:
    vector<cv::Point3f> world_corners;


    for(int h=0; h<6; h++)
    {
        for (int w=0; w<8; w++)
        {
            world_corners.push_back(cv::Point3f(27.f * w, 27.f * h, 0.f));
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
    calib_cam_series-=2;

    Mat board;
    Size boardSize(8,6);
    vector<vector<Point2f> > chessboardcorners(calib_cam_series);
    vector<vector<Point3f> > objectpoints;

    cout<<calib_cam_series<<endl;

    ///Find corners
    //#pragma omp parallel for
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

    Mat r, t, extrins;
    transpose(cam_rvecs[0], r);
    transpose(cam_tvecs[0], t);
    transpose(cam_dist.clone(), cam_dist);

    extrins = Mat(2,3, CV_64FC1);
    r.copyTo(extrins.row(0));
    t.copyTo(extrins.row(1));

    ///Make rvecs and tvecs into one Mat
    Mat rvec = Mat(3, cam_rvecs.size(), CV_64FC1);
    Mat tvec = Mat(3, cam_tvecs.size(), CV_64FC1);

    for(int i = 0; i< cam_rvecs.size(); i++)
    {
        //printmat(cam_tvecs[1], "ene tvec");
        rvec.at<double>(0,i) = cam_rvecs[i].at<double>(0,0);
        rvec.at<double>(1,i) = cam_rvecs[i].at<double>(0,1);
        rvec.at<double>(2,i) = cam_rvecs[i].at<double>(0,2);
    }

    for(int i = 0; i< cam_tvecs.size(); i++)
    {
        //printmat(cam_tvecs[1], "ene tvec");
        tvec.at<double>(0,i) = cam_tvecs[i].at<double>(0,0);
        tvec.at<double>(1,i) = cam_tvecs[i].at<double>(1,0);
        tvec.at<double>(2,i) = cam_tvecs[i].at<double>(2,0);
    }

    FileStorage fs("./camera.xml", FileStorage::WRITE);
    fs << "reprojection_error" << cam_error;
    fs << "camera_matrix" << cam_mat;
    fs << "distortion_camera" << cam_dist;
    fs << "extrinsic"<< extrins;
    fs << "rvec" << rvec;
    fs << "tvec" << tvec;
    fs.release();
}

Mat calculate3D_single_camera()
{
    FileStorage fs("./camera.xml", FileStorage::READ);
    Mat rvecs, tvecs;

    fs["rvec"] >> rvecs;
    fs["tvec"] >> tvecs;

    if(rvecs.cols != tvecs.cols)
    {
        cerr<<"Number of rvecs doesn't match number of tvecs"<<endl;
        exit(-1);
    }

    Mat result;
    for(int i=0; i<1/*rvecs.cols*/; i++)
    {
        Mat rvec = Mat(3,1, CV_64FC1);
        rvecs.col(i).copyTo(rvec.col(0));

        Mat tvec = Mat(3,1, CV_64FC1);
        tvecs.col(i).copyTo(tvec.col(0));

        Mat rotmat;
        Rodrigues(rvec, rotmat);

        Mat projmat = Mat::eye(Size(4,3), CV_64FC1);

        for(int i=0; i<projmat.rows; i++)
        {
            for(int j=0; j<projmat.cols; j++)
            {
                if(j<3)
                {
                    projmat.at<double>(i,j)=rotmat.at<double>(i,j);
                }
                else
                {
                    projmat.at<double>(i,j)=tvec.at<double>(i,0);
                }
            }
        }
        projmat.inv(DECOMP_SVD);
        transpose(projmat.clone(), projmat);

        Mat image = Mat(3, 1280*1025, CV_64FC1);
        float tellery = 0;
        float tellerx = 0;
        //cout<<image.cols<<endl;
        for(int j=0; j<image.cols; j++)
        {
            if(tellery < 1024)
            {
                image.at<double>(0, j) = tellerx;
                image.at<double>(1, j) = tellery;
                image.at<double>(2, j) = 1;
                //cout<<tellerx<<" "<<tellery<<" "<<1<<endl;
                //cout<<j<<endl;
                tellery++;
            }
            else
            {
                tellery=0;
                tellerx++;
            }
        }
        result = projmat * image;
        //cout<<"result"<<result.rows<<"x"<<result.cols<<endl;
        save(result);
        //printmat(image, "is zien");
    }



    return result;
}


