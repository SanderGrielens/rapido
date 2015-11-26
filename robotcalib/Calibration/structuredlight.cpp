#include "calibration.hpp"

using namespace AVT;
using namespace VmbAPI;

int NOP_v;
int NOP_h;
int camera_width;
int camera_height;

Decoder init_decoder()
{
    Decoder d;

    Mat newmat = Mat::zeros(camera_height, camera_width, CV_32FC1);

    ///initialize Decoder
    d.minimum.push_back(newmat.clone());
    d.minimum.push_back(newmat.clone());
    d.maximum.push_back(newmat.clone());
    d.maximum.push_back(newmat.clone());
    d.Ld.push_back(newmat.clone());
    d.Ld.push_back(newmat.clone());
    d.Lg.push_back(newmat.clone());
    d.Lg.push_back(newmat.clone());
    d.pattern_image.push_back(newmat.clone());
    d.pattern_image.push_back(newmat.clone());

    return d;
}

vector<Mat> generate_pattern(int NOP_v, int NOP_h, int projector_width, int projector_height)
{
    vector<Mat> result;

    for(int i =0 ; i< NOP_h*2 + NOP_v*2 + 4; i++)
    {
        Mat newmat = Mat(projector_height, projector_width, CV_8UC1);
        result.push_back(newmat) ;
    }
    ///Generate vertical patterns
    int teller =0;
    for(int k=NOP_v; k>=0; teller+=2, k--)
    {
        bool change = true;
        bool flag = true;
        for(int i = 0; i < projector_width; i++)
        {
            for(int j =0; j< projector_height; j++)
            {
                ///Fill in the column
                uchar pixel_color=0;
                if(flag)
                    pixel_color = 255;

                result[teller].at<uchar>( j, i ) = pixel_color;
                ///inverse

                if( pixel_color > 0 )
                    pixel_color = ( uchar ) 0;
                else
                    pixel_color = ( uchar ) 255;

                result[teller + 1].at<uchar>( j,i ) = pixel_color;  // inverse
            }

            int macht = pow(2, k);

            if(i%macht == 0 && i != 0)
            {
                if(change)
                {
                    flag = !flag;
                    change = false;
                }
                else
                    change = true;
            }
        }
    }

    ///Generate Horizontal patterns
    for(int k=NOP_h-1; k>=0; teller +=2, k--)
    {
        bool change = true;
        bool flag = true;
        for(int i = 0; i < projector_height; i++)
        {
            for(int j =0; j< projector_width; j++)
            {
                ///Fill in the row
                uchar pixel_color=0;
                if(flag)
                    pixel_color = 255;
                result[teller].at<uchar>( i, j ) = pixel_color;

                ///inverse
                if( pixel_color > 0 )
                    pixel_color = ( uchar ) 0;
                else
                    pixel_color = ( uchar ) 255;

                result[teller + 1].at<uchar>( i, j ) = pixel_color;  // inverse
            }
            int macht = pow(2,k);
            if(i%macht == 0 && i != 0)
            {
                if(change)
                {
                    flag = !flag;
                    change = false;
                }
                else
                    change = true;
            }
        }
    }
    return result;
}

bool get_sl_images(int delay, string path, int serie, int width, int height)
{
    vector<Mat> pattern;
    cout<<"serienummer: "<<serie<<endl;

    int vertical_patterns=0;
    ///Get dimensions
    while(pow(2, vertical_patterns) < width)
    {
        vertical_patterns++;
    }
    NOP_v = vertical_patterns;

    ///Do the same for the horizontal patterns
    int horizontal_patterns=0;
    while(pow(2, horizontal_patterns) < height)
    {
        horizontal_patterns++;
    }
    NOP_h = horizontal_patterns;

    CameraPtr camera ;
    FramePtr frame;
    char * pCameraID = NULL;

    VimbaSystem & system = VimbaSystem :: GetInstance ();
    if ( !VmbErrorSuccess == system.Startup () )
    {
        cout<<"system startup failed"<<endl;
    }
    else
        cout<<"system started"<<endl;

    std::string strCameraID;
    if(NULL == pCameraID)
    {
        CameraPtrVector cameras;
        system.GetCameras(cameras);
        if(cameras.size() <= 0)
        {
            cout<<"no camera found"<<endl;
            return false;
        }
        else
        {
            cameras[0]->GetID(strCameraID);
        }
    }
    else
    {
        strCameraID = pCameraID;
    }

    VmbErrorType res = system.OpenCameraByID(strCameraID.c_str(), VmbAccessModeFull, camera);
    if(VmbErrorSuccess == res)
        cout<<"Camera geopend"<<endl;

    pattern = generate_pattern(NOP_v, NOP_h, width, height);
    cout<<"Pattern generated"<<endl;

    int number_of_patterns = (NOP_h + NOP_v)*2;

    ///Save patterns
    /*vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION );
    //Kies 0 om geen compressie door te voeren
    compression_params.push_back(0);

    for(int i=0; i<number_of_patterns; i++)
    {
        ostringstream stm ;
        stm << i ;
        try {
            imwrite("patroon" + stm.str()+ ".png", pattern[i], compression_params);
        }
        catch (int runtime_error){
            fprintf(stderr, "Exception converting image to JPPEG format: %s\n");
            return 1;
        }
    }*/

    for(int i = 0; i<number_of_patterns; i++)
    {
        string Result;
        ostringstream convert;
        convert << i;
        ///Project pattern full screen via projector
        namedWindow( "pattern", CV_WINDOW_NORMAL );
        setWindowProperty("pattern", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
        moveWindow("pattern", 0, 0);
        imshow("pattern", pattern[i]);
        waitKey(delay);

        ///Read from camera
        FeaturePtr pCommandFeature;
        res = camera->GetFeatureByName("GVSPAdjustPacketSize", pCommandFeature );
        if ( VmbErrorSuccess == pCommandFeature->RunCommand() )
        {
            bool bIsCommandDone = false;
            do
            {
                if ( VmbErrorSuccess != pCommandFeature->IsCommandDone( bIsCommandDone ))
                {
                    break;
                }
            }
            while ( false == bIsCommandDone );
        }

        FeaturePtr pFormatFeature;
        // Set pixel format. For the sake of simplicity we only support Mono and BGR in this example.
        res = camera->GetFeatureByName( "PixelFormat", pFormatFeature );
        if ( VmbErrorSuccess == res )
        {
            // Try to set BGR
            res = pFormatFeature->SetValue( VmbPixelFormatRgb8 );
            if ( VmbErrorSuccess != res )
            {
                // Fall back to Mono
                res = pFormatFeature->SetValue( VmbPixelFormatMono8 );
            }

            if ( VmbErrorSuccess == res )
            {
                // Acquire
                res = camera->AcquireSingleImage( frame, 5000 );
            }
        }

        VmbUint32_t nImageSize = 0;
        VmbErrorType err = frame->GetImageSize( nImageSize );

        VmbUint32_t nWidth = 0;
        err = frame->GetWidth( nWidth );

        VmbUint32_t nHeight = 0;
        err = frame->GetHeight( nHeight );

        VmbUchar_t *pImage = NULL;
        err = frame->GetImage(pImage );

        AVTBitmap bitmap;
        bitmap.colorCode = ColorCodeMono8;
        bitmap.bufferSize = nImageSize;
        bitmap.width = nWidth;
        bitmap.height = nHeight;
        AVTCreateBitmap( &bitmap, pImage );

        string pFileName;
        ///Save camera image
        pFileName = path + "/frame" + convert.str()+ ".bmp";

        AVTWriteBitmapToFile( &bitmap, pFileName.c_str() );
    }

    return true;
}

bool findcorners(vector<vector<Point2f> > &chessboardcorners, string path, int aantalseries, int width, int height)
{
    Mat board;
    Size boardSize(8,6);

    int vertical_patterns=0;
    ///Get dimensions
    while(pow(2, vertical_patterns) < width)
    {
        vertical_patterns++;
    }
    NOP_v = vertical_patterns;

    ///Do the same for the horizontal patterns
    int horizontal_patterns=0;
    while(pow(2, horizontal_patterns) < height)
    {
        horizontal_patterns++;
    }
    NOP_h = horizontal_patterns;

    for(int i = 0; i<aantalseries; i++)
    {
        cout<<"finding corners image "<<i<<endl;
        ostringstream conv;
        conv << i;

        string pad = path + conv.str()+"/frame0.bmp";
        board = imread(pad, 0);
        if(board.empty())
        {
            cout<<"No image found"<<i<<endl;
            continue;
        }

        bool found1_1 = findChessboardCorners(board, boardSize, chessboardcorners[i],CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

        if(!found1_1)
        {
            std::cerr << "Checkboard 1_"<<i<<" corners not found!" << std::endl;
            return false;
        }
        cornerSubPix(board, chessboardcorners[i], Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 300, 0.001));
        cvtColor(board, board, CV_GRAY2BGR);
        drawChessboardCorners( board, boardSize, chessboardcorners[i], found1_1 );
        //tonen(board, "zijn ze gevonden?");
    }
    return true;
}

int check_bit(float value1, float value2, float Ld, float Lg, float m)
{
    //cout<<value1<<" "<<value2<<" "<<Ld<<" "<<Lg<<endl;
    if (Ld < m)
    {
        return 2;
    }
    if (Ld>Lg)
    {
        if( value1>value2)
            return 1;
        else
            return 0;
    }
    if (value1<=Ld && value2>=Lg)
    {
        return 0;
    }
    if (value1>=Lg && value2<=Ld)
    {
        return 1;
    }

    return 2;
}

void calculate_light_components(Decoder &d, vector<Mat> beelden, int dir, float b)
{
    int x = beelden[0].cols;
    int y = beelden[0].rows;
    int NOP;
    string richting;
    if(dir)
    {
        NOP = 2*NOP_v-1; //number of highest frequency vertical pattern
        richting="vert";
    }
    else
    {
        NOP = 2*NOP_v + 2*NOP_h-3; //number of highest frequency horizontal pattern
        richting="hor";
    }
    string LD = "ld"+ richting;
    string LG = "lg"+richting;

    for(int i =0; i< y; i++)
    {
        for(int j =0; j<x; j++)
        {
            float lpmax=0.0;
            float lpmin=1000.0;
            for(int k = 2; k< 8; k++)
            {
                if(beelden[NOP-k].at<float>(i,j) < lpmin)
                    lpmin = beelden[NOP-k].at<float>(i,j);
            }

            for(int k = 2; k < 8; k++)
            {
                if(beelden[NOP-k].at<float>(i,j) > lpmax)
                    lpmax = beelden[NOP-k].at<float>(i,j);
            }

            float _d =(lpmax - lpmin) / (1 - b);
            float g = (2*(lpmin - b*lpmax) / (1- pow(b,2)));

            d.Ld[dir].at<float>(i,j) = (g>0 ? _d : lpmax);
            d.Lg[dir].at<float>(i,j) = (g>0 ? g : 0);
        }
    }

    //tonen(d.Ld[dir]/255, "Ld");
    //tonen(d.Lg[dir]/255, "Lg");

    cout<<"direct and global light calculated"<<endl;
}

void get_pattern_image(Decoder &d, vector<Mat> beelden, int dir, float m, float thresh)
{
    int holder;
    int bit;
    int NOP;
    int start;
    string richting;
    float min_slecht = pow(2, NOP_v+2);

    if(dir)
    {
        start = 0;
        holder = NOP_v;
        NOP = 2*NOP_v-3; //number of highest frequency vertical pattern
        richting = "vert";
    }
    else
    {
        holder = NOP_h;
        NOP = 2*NOP_v + 2*NOP_h-3; //number of highest frequency horizontal pattern
        start = 2*NOP_v;
        richting = "hor";
    }

    bit = holder;
    //cout<<"bit: "<<bit<<endl;

    for(int j =0; j< beelden[NOP].rows; j++)
    {
        for(int k =0; k<beelden[NOP].cols; k++)
        {
            float val1 = beelden[NOP].at<float>(j,k);
            float val2 = beelden[NOP-1].at<float>(j,k);

            ///save min and max of every pixel in every image pair
            if(val1 < d.minimum[dir].at<float>(j,k) || val2 < d.minimum[dir].at<float>(j,k))
                d.minimum[dir].at<float>(j,k) = (val1<val2 ? val1 : val2);

            if(val1> d.maximum[dir].at<float>(j,k) || val2> d.maximum[dir].at<float>(j,k))
                d.maximum[dir].at<float>(j,k) = (val1>val2 ? val1 : val2);
        }
    }

    ///Go over each image pair (img and his inverse) and check for each pixel it's value. Add it to pattern[dir]
    /// If a pixel is in a lighted area, we add 2 raised to the power of (k - the frame number). This way we get a gray code pattern for each pixel in pattern[dir].
    /// Pattern[dir] is of type 32 bit float. If you have more than 32 patterns, change the type of pattern[dir].
    //#pragma omp parallel for
    for(int i = start; i<=NOP ; i+=2)
    {
        bit--;

        for(int j =0; j< beelden[i+1].rows; j++)
        {
            for(int k =0; k<beelden[i+1].cols; k++)
            {
                float val1 = beelden[i+1].at<float>(j,k);
                float val2 = beelden[i].at<float>(j,k);
                float ld = d.Ld[dir].at<float>(j,k);
                float lg = d.Lg[dir].at<float>(j,k);

                ///build pattern
                /*if (d.pattern_image[dir].at<float>(j,k) < max_goed)
                {*/
                    int p = check_bit(val1, val2, ld, lg, m);
                    if(p == 2)
                    {
                        d.pattern_image[dir].at<float>(j,k) += (p<<(NOP_v+2)); //p*(pow(2, NOP_v+2));
                    }
                    else
                    {
                        d.pattern_image[dir].at<float>(j,k) +=  (p<<bit); //p*(pow(2, bit));
                    }
                //}
            }
        }
    }


    ///Convert pattern from gray code to binary
    for(int i=0;i<d.pattern_image[dir].rows;i++)
    {
        for(int j=0; j<d.pattern_image[dir].cols;j++)
        {
            if (d.pattern_image[dir].at<float>(i,j) < min_slecht )
            {   //invalid value: use grey
                int q = static_cast<int>(d.pattern_image[dir].at<float>(i,j));
                int p = q;

                for (unsigned shift = 1; shift < holder; shift <<= 1)
                {
                    p ^= p >> shift;
                }
                d.pattern_image[dir].at<float>(i,j) = p + (d.pattern_image[dir].at<float>(i,j) - q);
                if(dir)
                    d.pattern_image[dir].at<float>(i,j)--;
            }

            if((d.maximum[dir].at<float>(i,j) - d.minimum[dir].at<float>(i,j)) < thresh)
            {
                d.pattern_image[dir].at<float>(i,j) = 2 << (NOP_v+2);// 2*(pow(2, NOP_v+2));
            }
        }
    }
    //tonen(d.minimum[dir], "minimum");
    //tonen(d.maximum[dir], "maximum");
    cout<<"pattern decoded"<<endl;
}

void colorize_pattern(Decoder &d, vector<Mat> &image, int dir, int projector_width, int projector_height)
{
    int NOP;

    int effective_width = projector_width;
    int effective_height = projector_height;
    if(dir)
    {
        int max_vert_value = (1<<NOP_v);
        while (effective_width>max_vert_value )
        {
            effective_width >>= 1;
        }
        NOP = effective_width;
    }
    else
    {
        int max_hor_value = (1<<NOP_h);
        while (effective_height>max_hor_value )
        {
            effective_height >>= 1;
        }
        NOP = effective_height;
    }

    float max_t = NOP;
    float n = 4.f;
    float dt = 255.f/n;

    for(int i = 0; i< d.pattern_image[dir].rows; i++)
    {
        for(int j = 0; j< d.pattern_image[dir].cols; j++)
        {
            if (d.pattern_image[dir].at<float>(i,j) >= (pow(2, NOP_v+2)))
            {   //invalid value: use grey
                image[dir].at<Vec3b>(i,j) = Vec3b(128, 128, 128);
                continue;
            }

            //display
            float t = d.pattern_image[dir].at<float>(i,j)*255.f/max_t;
            float c1 = 0.f, c2 = 0.f, c3 = 0.f;
            if (t<=1.f*dt)
            {   //black -> red
                float c = n*(t-0.f*dt);
                c1 = c;     //0-255
                c2 = 0.f;   //0
                c3 = 0.f;   //0
            }
            else if (t<=2.f*dt)
            {   //red -> red,green
                float c = n*(t-1.f*dt);
                c1 = 255.f; //255
                c2 = c;     //0-255
                c3 = 0.f;   //0
            }
            else if (t<=3.f*dt)
            {   //red,green -> green
                float c = n*(t-2.f*dt);
                c1 = 255.f-c;   //255-0
                c2 = 255.f;     //255
                c3 = 0.f;       //0
            }
            else if (t<=4.f*dt)
            {   //green -> blue
                float c = n*(t-3.f*dt);
                c1 = 0.f;       //0
                c2 = 255.f-c;   //255-0
                c3 = c;         //0-255
            }
            image[dir].at<Vec3b>(i,j) = Vec3b(c3, c2,c1);
        }
    }
    if(dir)
        tonen(image[dir], "kleurver");
    else
        tonen(image[dir], "kleurhor");

}

bool decode(int serienummer, Decoder &d, bool draw, string path, float b, float m, float thresh, int p_w, int p_h)
{
    vector<Mat> beelden ;
    ostringstream serienr;
    serienr << serienummer;
    string pad;
    string einde = ".bmp";

    pad = path + serienr.str() + "/frame";

    ///Reading every image in a serie
    for(int i=2; i<(NOP_v+NOP_h)*2; i++)
    {
        ostringstream beeldnr;
        beeldnr << i;
        Mat newmat_i = imread(pad + beeldnr.str() + einde,0);
        if(newmat_i.empty())
            cout<<"no image"<<endl;

        Mat newmat_float;
        newmat_i.convertTo(newmat_float, CV_32FC1);
        beelden.push_back(newmat_float);
    }


    camera_width = beelden[0].cols;
    camera_height = beelden[0].rows;

    d = init_decoder();

    Mat newmat2 = Mat::zeros(camera_height, camera_width, CV_8UC3);
    vector<Mat> image;
    image.push_back(newmat2.clone());
    image.push_back(newmat2.clone());

    ///Get vertical
    cout<<"Vertical: "<<endl;
    int vertical = 1;
    calculate_light_components(d, beelden, vertical, b);
    get_pattern_image(d, beelden, vertical, m, thresh);


    ///Get Horizontal
    cout<<"Horizontal: "<<endl;
    int horizontal = 0;
    calculate_light_components(d, beelden, horizontal, b);
    get_pattern_image(d, beelden, horizontal, m, thresh);

    ///Draw patterns?
    if(draw)
    {
        colorize_pattern(d, image, horizontal, p_w, p_h);
        colorize_pattern(d, image, vertical, p_w, p_h);
    }
    return true;
}

bool decode_all(int aantalseries, vector<Decoder> &dec, bool draw, string path, float b, float m, float thresh, int projector_width, int projector_height)
{
    bool gelukt;

    for(int i=0; i< aantalseries; i++)
    {
        Decoder d;
        cout<<"decode serie "<<i<<endl;
        gelukt = decode(i, d, draw, path, b, m, thresh, projector_width, projector_height);
        if(!gelukt)
            break;

        dec.push_back(d);
        cout<<endl;
    }
    if(!gelukt)
        return false;

    return true;
}

bool calibrate_sl(vector<Decoder> dec, vector<vector<Point2f> > corners, int aantalseries, int projector_width, int projector_height)
{
    clock_t time1 = clock();
    vector<vector<Point2f> > pcorners;
    int window = 10; ///the value for window is halve the size of the desired homography window

    vector<vector<Point3f> > objectpoints;
    vector<cv::Point3f> world_corners;


    for(int h=0; h<6; h++)
    {
        for (int w=0; w<8; w++)
        {
            world_corners.push_back(cv::Point3f(28.55555555f * w, 28.55555555f * h, 0.f));
        }
    }

    for(int k=0; k<aantalseries;k++)
    {
        Decoder d = dec[k];
        vector<Point2f> c = corners[k];
        vector<Point2f> proj;

        for(uint l = 0; l< c.size(); l++) //for each chessboard corner
        {
            vector<Point2f> cam_points;
            vector<Point2f> proj_points;
            if(c[l].x >= window && c[l].y >= window && c[l].x + window < camera_width && c[l].y + window<camera_height)
            {
                for(int x = c[l].x - window; x<=c[l].x + window; x++)  //for each point in the window around the corner
                {
                    for(int y = c[l].y - window; y<=c[l].y + window; y++)
                    {
                        if (d.pattern_image[0].at<float>(y,x) >= (pow(2, NOP_v+2)) || d.pattern_image[1].at<float>(y,x) >= (pow(2, NOP_v+2)))
                        {
                            continue;
                        }
                        else
                        {
                            cam_points.push_back(Point2f(x,y));
                            proj_points.push_back(Point2f(d.pattern_image[1].at<float>(y,x), d.pattern_image[0].at<float>(y,x))); ///pattern point is (vertical pattern, horizontal pattern)
                        }
                    }
                }
                Mat homography = findHomography(cam_points, proj_points, CV_RANSAC, 10 );
                Point3d p = Point3d(c[l].x, c[l].y, 1.0);
                Point3d Q = Point3d(Mat(homography*Mat(p)));
                Point2f q = Point2f(Q.x/Q.z, Q.y/Q.z);
                proj.push_back(q);
            }
            else
            {
                cout<<"chessboard corners are too close to the edge, fatal error"<<endl;
                exit(-1);
            }
        }

        //cornerSubPix(d.pattern_image[0], proj, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 300, 0.001));

        objectpoints.push_back(world_corners);
        pcorners.push_back(proj);
    }

    int cal_flags = 0
                //+ CV_CALIB_FIX_PRINCIPAL_POINT
                //+ cv::CALIB_FIX_K1
                //+ cv::CALIB_FIX_K2
                //+ cv::CALIB_ZERO_TANGENT_DIST
                + cv::CALIB_FIX_K3
                ;

    Size imageSize = dec[0].pattern_image[0].size();
    vector<Mat> cam_rvecs, cam_tvecs;
    Mat cam_mat;
    Mat cam_dist;
    int cam_flags = cal_flags;
    double cam_error = cv::calibrateCamera(objectpoints, corners, imageSize, cam_mat, cam_dist, cam_rvecs, cam_tvecs, cam_flags,
                                            TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5));

    cout<<"camera calibration RMS error: "<<cam_error<<endl;

    //calibrate the projector ////////////////////////////////////
    vector<cv::Mat> proj_rvecs, proj_tvecs;
    Mat proj_mat;
    Mat proj_dist;
    int proj_flags = cal_flags;
    Size projector_size(projector_width, projector_height);
    double proj_error = cv::calibrateCamera(objectpoints, pcorners, projector_size, proj_mat, proj_dist, proj_rvecs, proj_tvecs, proj_flags,
                                             TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5));
    cout<<"projector calibration RMS error: "<<proj_error<<endl;

    //stereo calibration
    Mat R, T, E, F;
    double stereo_error = cv::stereoCalibrate(objectpoints, corners, pcorners, cam_mat, cam_dist, proj_mat, proj_dist,
                                                imageSize , R, T, E, F,
                                                //CV_CALIB_FIX_INTRINSIC+
                                                CV_CALIB_USE_INTRINSIC_GUESS
                                                //CV_CALIB_FIX_ASPECT_RATIO +
                                                //CV_CALIB_ZERO_TANGENT_DIST +
                                                //CV_CALIB_SAME_FOCAL_LENGTH +
                                                //CV_CALIB_RATIONAL_MODEL +
                                                /*CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5,
                                                TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5)*/);
    cout<<"Stereo RMS error: "<<stereo_error<<endl;
    clock_t time2 = clock();
    cout<<"tijd calib: "<< (float)(time2-time1)/CLOCKS_PER_SEC<<endl;
    ///Save calibration:

    FileStorage fs("./calib_sl/camera_projector.xml", FileStorage::WRITE);
    fs << "fundamental" << F;
    fs << "rotation" << R;
    fs << "translation" << T;
    fs << "essential" << E;
    fs << "camera_matrix" << cam_mat;
    fs << "projector_matrix" << proj_mat;
    fs << "distortion_camera" << cam_dist;
    fs << "distorion_projector" << proj_dist;
    fs.release();

    return true;
}

vector<Visualizer> calculate3DPoints_all(string path, int aantalseries, float b, float m, float thresh, int projector_width, int projector_height, vector<Point2f> &c)
{
    bool draw = false;
    vector<Decoder> dec;
    vector<Visualizer> viz;

    decode_all(aantalseries, dec, draw, path, b, m, thresh, projector_width, projector_height);

        ///Get intrinsic and extrinsic camera parameters
    FileStorage fs("./calib_sl/camera_projector.xml", FileStorage::READ);
    Mat cameraMatrix;
    Mat projMatrix;
    Mat camdist;
    Mat projdist;
    Mat came = Mat(1234,1626, CV_32FC2);
    Mat cam_uc, cam_up;

    for(int i = 0; i<1626; i++) //x
    {
        for(int j = 0; j< 1234; j++) //y
        {
            came.at<Vec2f>(j,i)[1] = j; // y-channel
            came.at<Vec2f>(j,i)[0] = i; // x -channel
            //cout<<i<<" "<<j<<" "<<came.at<Vec2f>(j,i)[0]<<" "<<came.at<Vec2f>(j,i)[1]<<endl;
        }
    }

    Mat rotMat;
    Mat transMat;

    fs["rotation"] >> rotMat;
    if(rotMat.empty())
        cout<<"rotleeg"<<endl;

    fs["translation"] >> transMat;

    if(transMat.empty())
        cout<<"tranleeg"<<endl;

    Mat projmat1 = Mat::eye(Size(4,3), CV_64F);
    for(int i=0; i<projmat1.rows; i++)
    {
        for(int j=0; j<projmat1.cols; j++)
        {
            if(j==i && j<3)
            {
                projmat1.at<double>(i,j)=1;
            }
            else
            {
                projmat1.at<double>(i,j)=0;
            }
        }
    }

    Mat projmat2 = Mat::eye(Size(4,3), CV_64F);
    for(int i=0; i<projmat2.rows; i++)
    {
        for(int j=0; j<projmat2.cols; j++)
        {
            if(j<3)
            {
                projmat2.at<double>(i,j)=rotMat.at<double>(i,j);
            }
            else
            {
                projmat2.at<double>(i,j)=transMat.at<double>(i,0);
            }
        }
    }

    fs["camera_matrix"] >> cameraMatrix;
    fs["projector_matrix"] >> projMatrix;

    fs["distortion_camera"] >> camdist;
    fs["distorion_projector"] >> projdist;

    if(!came.empty())
    {
        undistort(came.clone(), cam_uc, cameraMatrix, camdist);
        undistort(came.clone(), cam_up, projMatrix, projdist);
    }
    else
    {
        cout<<"No data to undistort"<<endl;
        exit(-1);
    }

    //#pragma omp parallel for
    for(int k = 0; k< aantalseries; k++)
    {
        Decoder d = dec[k];
        Visualizer v;
        vector<Point2d> c_p;
        vector<Point2d> p_p;
        vector<Point2d> c_pp;
        vector<Point2d> p_pp;
        vector<Point2d> cam_points;
        vector<Point2d> proj_points;
        vector<Point2d> un;
        Mat rechts = Mat::zeros(camera_height, camera_width, CV_8UC1);
        Mat beneden = rechts.clone();
        Mat hor = d.pattern_image[0]; //gives y coordinate(variates in Y direction)
        Mat ver = d.pattern_image[1]; //gives x coordinate(variates in x direction)
        //GaussianBlur(hor, hor, Size(1,5), 1.5, 0);
        //GaussianBlur(ver, ver, Size(5,1), 1.5, 0);
        int teller =0;
        //#pragma omp parallel for
        for(int x = 0; x<camera_width; x++)
        {
            for(int y = 0; y<camera_height; y++)
            {
                int tel = 1;
                if (hor.at<float>(y,x) >= (pow(2, NOP_v+2)) || ver.at<float>(y,x) >= (pow(2, NOP_v+2)))
                {
                    continue;
                }
                if(hor.at<float>(y,x) == hor.at<float>(y+1, x))
                {
                    tel = 2;
                    while(hor.at<float>(y,x) == hor.at<float>(y+tel, x))
                    {
                        tel++;
                    }
                }

                beneden.at<uchar>(y,x) = tel;
                y+=tel;
            }
        }
        //#pragma omp parallel for
        for(int y = 0; y<camera_height; y++)
        {
            for(int x = 0; x<camera_width; x++)
            {
                int tel = 1;
                if (hor.at<float>(y,x) >= (pow(2, NOP_v+2)) || ver.at<float>(y,x) >= (pow(2, NOP_v+2)))
                {
                    continue;
                }
                if(ver.at<float>(y,x) == ver.at<float>(y, x+1))
                {
                    tel = 2;
                    while(ver.at<float>(y,x) == ver.at<float>(y, x+tel))
                    {
                        tel++;
                    }
                }

                rechts.at<uchar>(y,x) = tel;
                x+=tel;
            }
        }
        //#pragma omp parallel for
        for(int x = 0; x<camera_width; x++)
        {
            for(int y =0; y< camera_height; y++)
            {
                int onder = beneden.at<uchar>(y,x);
                int opzij = rechts.at<uchar>(y,x);
                if(onder >0 && opzij >0 && x+opzij/2 < camera_width && y + onder/2 <  camera_height)
                {
                    cam_points.push_back(Point2d(x + (opzij/2),y+(onder/2)));
                    Point2d punt = Point2d(ver.at<float>(y + (onder/2),x+(opzij/2)), hor.at<float>(y+(onder/2),x+(opzij/2)));
                    proj_points.push_back(punt);///pattern point (x,y) coordinates is (vertical pattern, horizontal pattern)
                }
                else
                    teller++;
            }
        }

        cout<<"gefilterde punten: "<<teller<<endl;

        ///Building 3D point cloud

        /*Mat cam, proj;
        cam = Mat(cam_points);
        proj = Mat(proj_points);*/
        //Mat driedpunten = Mat(1, cam_points.size(), CV_64FC4);
        //GaussianBlur(proj, proj, Size(5, 5), 1.5, 0);

        ///Home made projection matrices:
        Mat P0, P1;

        P0 = cameraMatrix * projmat1;
        P1 = projMatrix * projmat2;

        int number_of_cores= 16;
        omp_set_num_threads(number_of_cores);
        vector<vector<Point2d> > cams;
        vector<vector<Point2d> > projs;
        vector<Mat> driepunt;
        int deler = cam_points.size()/number_of_cores;

        teller = 0;
        int elementteller = 0;
        while(elementteller<cam_points.size())
        {
            vector<Point2d> cam;
            vector<Point2d> proj;
            for(int i= teller*deler; i<(teller + 1)*deler; i++)
            {
                if(i>= cam_points.size())
                    break;

                cam.push_back(cam_points[i]);
                proj.push_back(proj_points[i]);
                elementteller++;
            }
            cams.push_back(cam);
            projs.push_back(proj);
            teller++;
        }

        #pragma omp parallel for
        for(int i = 0; i< number_of_cores; i++)
        {
            Mat deel = Mat(1, cams[i].size(), CV_64FC4);
            Mat camsMat = Mat(cams[i]);
            Mat projsMat = Mat(projs[i]);

            clock_t time1 = clock();
            triangulatePoints(P0, P1, camsMat, projsMat, deel);
            clock_t time2 = clock();
            cout<<"tijd trianguleren "<<(float)(time2-time1)/CLOCKS_PER_SEC<<endl;
            cout<<"We hebben "<<cams[i].size()<<" punten getrianguleerd"<<endl;

            cout<<deel.rows<<" "<<deel.cols<<endl;
            driepunt.push_back(deel);
        }

        for(int i =0; i<driepunt.size(); i++)
        {
            cout<<"i: "<<i<<"\n"
                "Dims: "<<driepunt[i].dims<<"\n"
                "rows: "<<driepunt[i].rows<<"\n"
                "type: "<<driepunt[i].type()<<endl;
        }

        Mat driedpunten;// = Mat(1, cam_points.size(), CV_64FC4);
        hconcat(driepunt, driedpunten);
        cout<<"concat gelukt"<<driepunt[0].dims<<endl;
        v.pointcloud= driedpunten;
        v.cam_points = cam_points;
        viz.push_back(v);

        double X,Y,Z;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);//(new pcl::pointcloud<pcl::pointXYZ>);

        for(int i=0;i<driedpunten.cols;i++)
        {
            //std::cout<<i<<endl;
            X = driedpunten.at<double>(0,i) / driedpunten.at<double>(3,i);
            Y = driedpunten.at<double>(1,i) / driedpunten.at<double>(3,i);
            Z = driedpunten.at<double>(2,i) / driedpunten.at<double>(3,i);

            pcl::PointXYZRGB point;
            point.x = X;
            point.y = Y;
            point.z = Z;
            //cout<<"x: "<<point.x<<" y: "<<point.y<<" z: "<<point.z<<endl;
            uint8_t r = 255, g = 0, b = 0;    // Example: Red color
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            point.rgb = *reinterpret_cast<float*>(&rgb);
            point_cloud_ptr -> points.push_back(point);
        }


        point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
        point_cloud_ptr->height =1;
        pcl::PLYWriter plywriter;
        plywriter.write("./test.ply", *point_cloud_ptr, false);
        pcl::io::savePCDFileBinary("./test2.pcd", *point_cloud_ptr);
    }

    return viz;
}

Mat calculate3DPoints(vector<Point2f> &c, Decoder d)
{
    vector<Point2d> cam_points;
    vector<Point2d> proj_points;
    int window = 0;
    for(int l = 0; l< c.size(); l++) //for each chessboard corner
    {
        if (d.pattern_image[0].at<float>(c[l].y,c[l].x) >= (pow(2, NOP_v+2)) || d.pattern_image[1].at<float>(c[l].y,c[l].x) >= (pow(2, NOP_v+2)))
        {
            continue;
        }
        else
        {
            cam_points.push_back(Point2d(c[l].x,c[l].y));
            proj_points.push_back(Point2d(d.pattern_image[1].at<float>(c[l].y,c[l].x), d.pattern_image[0].at<float>(c[l].y,c[l].x))); ///pattern point is (vertical pattern, horizontal pattern)
        }
    }

    //cout<<"cam_points.size: "<<cam_points.size()<<endl;
    //cout<<"proj_points.size: "<<proj_points.size()<<endl;

        ///Get intrinsic and extrinsic camera parameters
    FileStorage fs("./calib_sl/camera_projector.xml", FileStorage::READ);
    Mat cameraMatrix;
    Mat projMatrix;
    Mat rotMat;
    Mat transMat;

    fs["camera_matrix"] >> cameraMatrix;
    fs["projector_matrix"] >> projMatrix;

    fs["rotation"] >> rotMat;
    if(rotMat.empty())
        cout<<"rotleeg"<<endl;

    fs["translation"] >> transMat;

    if(transMat.empty())
        cout<<"tranleeg"<<endl;

    Mat projmat1 = Mat::eye(Size(4,3), CV_64F);
    for(int i=0; i<projmat1.rows; i++)
    {
        for(int j=0; j<projmat1.cols; j++)
        {
            if(j==i && j<3)
            {
                projmat1.at<double>(i,j)=1;
            }
            else
            {
                projmat1.at<double>(i,j)=0;
            }
        }
    }

    Mat projmat2 = Mat::eye(Size(4,3), CV_64F);
    for(int i=0; i<projmat2.rows; i++)
    {
        for(int j=0; j<projmat2.cols; j++)
        {
            if(j<3)
            {
                projmat2.at<double>(i,j)=rotMat.at<double>(i,j);
            }
            else
            {
                projmat2.at<double>(i,j)=transMat.at<double>(i,0);
            }
        }
    }

    Mat cam, proj;
    cam = Mat(cam_points);
    proj = Mat(proj_points);
    Mat driedpunten = Mat(1, cam_points.size(), CV_64FC4);
    //GaussianBlur(proj, proj, Size(5, 5), 1.5, 0);

    ///Home made projection matrices:
    Mat P0, P1;

    P0 = cameraMatrix * projmat1;
    P1 = projMatrix * projmat2;

    //clock_t time3 = clock();
    triangulatePoints(P0, P1, cam, proj, driedpunten);
    //clock_t time4 = clock();
    //cout<<"tijd trianguleren "<<(float)(time4-time3)/CLOCKS_PER_SEC<<endl;
    cout<<"Triangulated "<<cam_points.size()<<" points"<<endl;


    double X,Y,Z;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);//(new pcl::pointcloud<pcl::pointXYZ>);
    //vector<Point3d> punten;
    Mat result = Mat(c.size(),4,CV_64FC1);
    for(int i=0;i<c.size();i++)
    {
        //std::cout<<i<<endl;
        X = driedpunten.at<double>(0,i) / driedpunten.at<double>(3,i);
        Y = driedpunten.at<double>(1,i) / driedpunten.at<double>(3,i);
        Z = driedpunten.at<double>(2,i) / driedpunten.at<double>(3,i);
        result.at<double>(i,0) = X;
        result.at<double>(i,1) = Y;
        result.at<double>(i,2) = Z;
        result.at<double>(i,3) = 1;

        pcl::PointXYZRGB point;
        point.x = X;
        point.y = Y;
        point.z = Z;
        //cout<<"x: "<<point.x<<" y: "<<point.y<<" z: "<<point.z<<endl;
        uint8_t r = 255, g = 0, b = 0;    // Example: Red color
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        point.rgb = *reinterpret_cast<float*>(&rgb);
        point_cloud_ptr -> points.push_back(point);

        point_cloud_ptr -> points.push_back(point);
    }

    return result;
    /*std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*point_cloud_ptr,*point_cloud_ptr, indices);
    point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
    point_cloud_ptr->height =1;
    pcl::PLYWriter plywriter;
    plywriter.write("./test.ply", *point_cloud_ptr, false);
    pcl::io::savePCDFileBinary("./test.pcd", *point_cloud_ptr);*/
}

Mat getRobotPoints(int height, int width)
{
    Mat result = Mat(height, width, CV_64FC1);

    ///We use a self constructed point vector, but in time we will read it from the robot
    double row = 5;
    double col = 0;
    for(int i =0; i< height; i++)
    {
        result.at<double>(i,0) = col*28.55555555;
        result.at<double>(i,1) = row*28.55555555;
        result.at<double>(i,2) = 0.5;//(row+col)/2*28.55555555;
        result.at<double>(i,3) = 1;
        if(col == 7)
        {
            col = 0;
            row--;
            continue;
        }
        col++;
    }
    return result;
}

Mat calculateTransMat(Mat origin, Mat dest)
{
    Mat transMat;
    Mat inliers;

    //Extract X, Y, Z coordinate from coordinates (discard W coordinate)
    Mat origin3d = Mat(origin.rows, 3, CV_64FC1);
    Mat dest3d= Mat(dest.rows, 3, CV_64FC1);
    for(int i = 0; i<origin.rows; i++)
    {
        for(int j=0; j<3; j++)
        {
            origin3d.at<double>(i,j) = origin.at<double>(i,j);
            dest3d.at<double>(i,j) = dest.at<double>(i,j);
        }
    }

    int result = estimateAffine3D(origin3d,dest3d, transMat, inliers);
    if(result>0)
        cout<<"Transformation matrix calculated."<<endl;
    else
        cout<<"Failed to calculate transformation matrix."<<endl;
    return transMat;
}

Mat convert(Mat origin, Mat transMat)
{
    transpose(origin.clone(), origin);
    Mat driedpunten = transMat*origin;
    return driedpunten;
}

void getRmsError(Mat transformed, Mat dest)
{
    transpose(transformed.clone(), transformed);
    double teller=0;
    for(int i=0;i<transformed.rows;i++)
    {
        double verschilx = abs(transformed.at<double>(i,0)) - abs(dest.at<double>(i,0));
        double verschily = abs(transformed.at<double>(i,1)) - abs(dest.at<double>(i,1));
        double verschilz = abs(transformed.at<double>(i,2)) - abs(dest.at<double>(i,2));
        double verschil = sqrt(pow(verschilx,2) + pow(verschily,2) + pow(verschilz,2));
        teller += pow(verschil,2);
    }

    double rms = sqrt(teller/transformed.rows);
    cout<<"rms error transformation = "<<rms<<" mm"<<endl;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

void save(Mat origin, Mat transformed, Mat dest)
{

    transpose(transformed.clone(), transformed);

    double X,Y,Z;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);//(new pcl::pointcloud<pcl::pointXYZ>);

    for(int i=0;i<transformed.rows;i++)
    {
        //std::cout<<i<<endl;
        X = transformed.at<double>(i,0);
        Y = transformed.at<double>(i,1);
        Z = transformed.at<double>(i,2);

        pcl::PointXYZRGB point;
        point.x = X;
        point.y = Y;
        point.z = Z;
        //cout<<"x: "<<point.x<<" y: "<<point.y<<" z: "<<point.z<<endl;
        uint8_t r = 255, g = 0, b = 0;    // Example: Red color
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        point.rgb = *reinterpret_cast<float*>(&rgb);
        point_cloud_ptr -> points.push_back(point);
    }

    for(int i=0;i<origin.rows;i++)
    {
        //std::cout<<i<<endl;
        X = origin.at<double>(i,0);
        Y = origin.at<double>(i,1);
        Z = origin.at<double>(i,2)-700;

        pcl::PointXYZRGB point;
        point.x = X;
        point.y = Y;
        point.z = Z;
        //cout<<"x: "<<point.x<<" y: "<<point.y<<" z: "<<point.z<<endl;
        uint8_t r = 0, g = 255, b = 0;
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        point.rgb = *reinterpret_cast<float*>(&rgb);
        point_cloud_ptr -> points.push_back(point);
    }

    for(int i=0;i<dest.rows;i++)
    {
        //std::cout<<i<<endl;
        X = dest.at<double>(i,0);
        Y = dest.at<double>(i,1);
        Z = dest.at<double>(i,2);

        pcl::PointXYZRGB point;
        point.x = X;
        point.y = Y;
        point.z = Z;
        //cout<<"x: "<<point.x<<" y: "<<point.y<<" z: "<<point.z<<endl;
        uint8_t r = 0, g = 0, b = 255;    // Example: Red color
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        point.rgb = *reinterpret_cast<float*>(&rgb);
        point_cloud_ptr -> points.push_back(point);
    }

    point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
    point_cloud_ptr->height =1;
    pcl::PLYWriter plywriter;
    plywriter.write("./test.ply", *point_cloud_ptr, false);
    pcl::io::savePCDFileBinary("./test.pcd", *point_cloud_ptr);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = rgbVis(point_cloud_ptr);
    while (!viewer->wasStopped ())
      {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      }
}

bool calibrate_sl_r(string path, float b, float m, float thresh, int projector_width, int projector_height)
{
    ///First, we'll use chessboardcorners as unique points to find
    vector <vector<Point2f> > chessboardcorners(1);
    bool gelukt_f = findcorners(chessboardcorners, path, 1, projector_width, projector_height);
    ///Get the pointcloud
    vector<Visualizer> viz =  calculate3DPoints_all(path, 1,  b, m, thresh, projector_width, projector_height, chessboardcorners[0]);
    /*///Calculate the 3D position of the chessboardcorners
    Mat points_sensor;
    bool draw = false;
    Decoder d;
    bool gelukt = decode(0, d, draw, path, b, m, thresh, projector_width, projector_height);
    points_sensor = calculate3DPoints(chessboardcorners[0], d); ///points_sensor = 48 x 4
    cout<<"sensor points acquired"<<endl;

    ///Move the robot arm to every corner and record its 3D position
    Mat points_robot = getRobotPoints(points_sensor.rows, points_sensor.cols); ///Points_robot = 48 x 4
    cout<<"Robot points acquired"<<endl;

    ///Calculate the transformation matrix (rotation matrix and translation vector) between the two coordinate systems

    Mat transMat = calculateTransMat(points_sensor, points_robot); ///transMat = 3 x 4

    ///Convert the sensor points to the coordinate system of the robot
    Mat transformed = convert(points_sensor, transMat); ///transformed = 3 x 48

    ///Print out the root mean square error
    getRmsError(transformed, points_robot);

    ///Save the data as a .pcd and .ply file
    save(points_sensor, transformed, points_robot);*/
    return true;
}
