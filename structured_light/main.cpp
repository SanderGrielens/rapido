#include "functies.hpp"

using namespace std;
using namespace cv;
using namespace AVT;
using namespace VmbAPI;

int NOP_h;
int NOP_v;
int aantalseries;
float b;
float m;
float thresh;
int projector_width;
int projector_height;
int camera_width;
int camera_height;

const float PIXEL_UNCERTAIN = std::numeric_limits<float>::quiet_NaN();
const unsigned short BIT_UNCERTAIN = 0xffff;

vector<Mat> generate_pattern()
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
        for(int i = 0; i < projector_width;i++)
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
        for(int i = 0; i < projector_height;i++)
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

bool get_calib_images(int delay, int serie)
{
    vector<Mat> pattern;
    cout<<"serienummer: "<<serie<<endl;
    ostringstream conv;
    conv << serie;
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

    pattern = generate_pattern();
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

    for(int i = 0; i<number_of_patterns;i++)
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
            } while ( false == bIsCommandDone );
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

        ///Save camera image
        string pFileName = "./picture/serie" + conv.str() + "/frame" + convert.str()+ ".bmp";
        AVTWriteBitmapToFile( &bitmap, pFileName.c_str() );

        /*///Save the first (and fully lighted) image to camera calibration folder.
        if(i==0)
        {
            string pFileName = "./calibration_camera/calib_serie" + conv.str() + ".bmp";
            AVTWriteBitmapToFile( &bitmap, pFileName.c_str() );
        }*/

    }

    return true;
}

bool findcorners(vector<vector<Point2f> > &chessboardcorners, int aantalseries)
{
    Mat board;
    Size boardSize(8,6);
    FileStorage fs("chessboardcorners.xml", FileStorage::READ);
    for(int i = 0; i<aantalseries; i++)
    {
        cout<<"nummer "<<i<<endl;
        ostringstream conv;
        conv << i;
        FileStorage fs("./picture/serie" + conv.str() + "/chessboardcorners.xml", FileStorage::READ);
        fs["corners"] >> chessboardcorners[i];
        if(chessboardcorners[i].empty())
        {
            fs.release();
            string path = "./picture/serie" + conv.str()+"/frame0.bmp";
            board = imread(path, 0);
            if(board.empty())
                cout<<"leeg"<<endl;

            //bool found1_1 = findChessboardCorners(board, boardSize, chessboardcorners[i],
                                                  //CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_NORMALIZE_IMAGE + CV_CALIB_CB_FILTER_QUADS);

            bool found1_1 = findChessboardCorners(board, boardSize, chessboardcorners[i],CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

            if(!found1_1)
            {
                std::cerr << "Checkboard 1_"<<i<<" corners not found!" << std::endl;
                return false;
            }
            cornerSubPix(board, chessboardcorners[i], Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 300, 0.001));
            cvtColor(board, board, CV_GRAY2BGR);
            drawChessboardCorners( board, boardSize, chessboardcorners[i], found1_1 );

            tonen(board, "zijn ze gevonden?");
            FileStorage fs("./picture/serie" + conv.str() + "/chessboardcorners.xml", FileStorage::WRITE);
            fs << "corners"<< chessboardcorners[i];
            fs.release();
        }
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

void calculate_light_components(Decoder &d, vector<Mat> beelden, int dir, string serienr)
{
    int minld = 1000;
    int maxld = 0;
    int minlg = 1000;
    int maxlg = 0;
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

    FileStorage fs("./picture/serie" + serienr + "/lightcomponents.xml", FileStorage::READ);
    fs[LD] >> d.Ld[dir];
    fs[LG] >> d.Lg[dir];
    fs.release();
    FileStorage fs1();

    if(d.Ld[dir].empty() || d.Lg[dir].empty())
    {
        d.Ld[dir] = Mat(beelden[NOP_v].rows, beelden[NOP_v].cols, CV_32FC1);
        d.Lg[dir] = Mat(beelden[NOP_v].rows, beelden[NOP_v].cols, CV_32FC1);
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
                //cout<<i<<" "<<j<<endl;
            }
        }


        //d.Lg[dir].convertTo(d.Lg[dir], CV_8UC1);
        //d.Ld[dir].convertTo(d.Ld[dir], CV_8UC1);

        FileStorage fs;
        if(dir)
             fs.open("./picture/serie" + serienr + "/lightcomponents.xml", FileStorage::WRITE);
        else
             fs.open("./picture/serie" + serienr + "/lightcomponents.xml", FileStorage::APPEND);

        if(!fs.isOpened())
            cout<<"file won't open"<<endl;


        fs << LD << d.Ld[dir];
        fs << LG << d.Lg[dir];
        fs.release();
    }

    //tonen(d.Ld[dir]/255, "Ld");
    //tonen(d.Lg[dir]/255, "Lg");

    cout<<"direct and global light calculated"<<endl;
}

void get_pattern_image(Decoder &d, vector<Mat> beelden, int dir, string serienr)
{
    int holder;
    int bit;
    int NOP;
    int start;
    string richting;

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

    FileStorage fs("./picture/serie" + serienr+ "/pattern.xml", FileStorage::READ);
    fs["pattern" + richting] >> d.pattern_image[dir];
    fs.release();

    if(d.pattern_image[dir].empty())
    {
        d.pattern_image[dir] = Mat::zeros(beelden[NOP_v].rows, beelden[NOP_v].cols, CV_32FC1);
        bit = holder;
        //cout<<"bit: "<<bit<<endl;

        ///Go over each image pair (img and his inverse) and check for each pixel it's value. Add it to pattern[dir]
        /// If a pixel is in a lighted area, we add 2 raised to the power of the frame. This way we get a gray code pattern for each pixel in pattern[dir].
        /// Pattern[dir] is of type 32 bit float. If you have more than 32 patterns, change the type of pattern[dir].
        for(int i = start; i<=NOP ; i+=2)
        {
            Mat img2 = beelden[i].clone();
            Mat img1 = beelden[i+1].clone();
            bit--;
            int teller = 0;

            for(int j =0; j< img1.rows; j++)
            {
                for(int k =0; k<img1.cols; k++)
                {
                    float val1 = img1.at<float>(j,k);
                    float val2 = img2.at<float>(j,k);
                    float ld = d.Ld[dir].at<float>(j,k);
                    float lg = d.Lg[dir].at<float>(j,k);

                    ///save min and max of every image pair
                    if(val1 < d.minimum[dir].at<float>(j,k) || val2 < d.minimum[dir].at<float>(j,k))
                        d.minimum[dir].at<float>(j,k) = (val1<val2 ? val1 : val2);

                    if(val1> d.maximum[dir].at<float>(j,k) || val2> d.maximum[dir].at<float>(j,k))
                        d.maximum[dir].at<float>(j,k) = (val1>val2 ? val1 : val2);

                    ///build pattern
                    if (d.pattern_image[dir].at<float>(j,k) < pow(2, NOP_v+1))
                    {
                        int p = check_bit(val1, val2, ld, lg, m);
                        if(p == 2)
                        {
                            d.pattern_image[dir].at<float>(j,k) += p*(pow(2, NOP_v+2));
                            teller++;
                        }
                        else
                        {
                            d.pattern_image[dir].at<float>(j,k) += p*(pow(2, bit));
                        }
                    }
                }
            }
        }
        ///Convert pattern from gray code to binary
        for(int i=0;i<d.pattern_image[dir].rows;i++)
        {
            for(int j=0; j<d.pattern_image[dir].cols;j++)
            {
                if (d.pattern_image[dir].at<float>(i,j) < (pow(2, NOP_v+2)) )
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
                    d.pattern_image[dir].at<float>(i,j) = 2*(pow(2, NOP_v+2));
                }
            }
        }

        FileStorage fs1;
        bool gelukt;

        if(dir)
        {
            gelukt = fs1.open("./picture/serie" + serienr+ "/pattern.xml", FileStorage::WRITE);
        }
        else
        {
            gelukt = fs1.open("./picture/serie" + serienr+ "/pattern.xml", FileStorage::APPEND);
        }
        if(!fs1.isOpened())
            cout<<"file won't open"<<endl;
        fs1<< "pattern" + richting<< d.pattern_image[dir];
        fs1.release();

        //tonen(d.minimum[dir], "minimum");
        //tonen(d.maximum[dir], "maximum");
    }
}

void colorize_pattern(Decoder &d, vector<Mat> &image, int dir)
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
    int set = 0;
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

bool decode(int serienummer, Decoder &d, bool draw)
{
    vector<Mat> beelden ;
    ostringstream serienr;
    serienr << serienummer;
    string path = "./picture/serie" + serienr.str() + "/frame";
    string einde = ".bmp";

    ///Reading every image in a serie
    for(int i=2; i<(NOP_v+NOP_h)*2; i++)
    {
        ostringstream beeldnr;
        beeldnr << i;
        Mat newmat_i = imread(path + beeldnr.str() + einde,0);
        if(newmat_i.empty())
            cout<<"no image"<<endl;

        Mat newmat_float;
        newmat_i.convertTo(newmat_float, CV_32FC1);
        beelden.push_back(newmat_float);
    }

    camera_width = beelden[0].cols;
    camera_height = beelden[0].rows;

    ///Calculate the matrices Ld and Lg of the series (Direct light and Global Light respectively)
    Mat newmat = Mat(beelden[NOP_v].rows, beelden[NOP_v].cols, CV_32FC1);
    Mat newmat2 = Mat(beelden[1].size(), CV_8UC3);

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

    vector<Mat> image;
    image.push_back(newmat2.clone());
    image.push_back(newmat2.clone());

    cout<<"Vertical: "<<endl;

    ///Get vertical
    int vertical = 1;
    calculate_light_components(d, beelden, vertical, serienr.str());
    get_pattern_image(d, beelden, vertical, serienr.str());

    cout<<"Horizontal: "<<endl;

    ///Get Horizontal
    int horizontal = 0;
    calculate_light_components(d, beelden, horizontal, serienr.str());
    get_pattern_image(d, beelden, horizontal, serienr.str());

    ///draw?
    if(draw)
    {
        colorize_pattern(d, image, horizontal);
        colorize_pattern(d, image, vertical);
    }
    return true;
}

bool decode_all(int aantalseries, vector<Decoder> &dec, bool draw)
{
    bool gelukt;
    for(int i=0; i< aantalseries; i++)
    {
        Decoder d;
        cout<<"decode "<<i<<endl;
        cout<<endl;
        gelukt = decode(i, d, draw);
        if(!gelukt)
            return false;

        dec.push_back(d);
    }

    return true;
}

bool calibrate(vector<Decoder> dec, vector<vector<Point2f> > corners, int aantalseries)
{
    vector<vector<Point2f> > pcorners;
    int window = 10; ///the value for window is halve the size of the desired homography window

    vector<vector<Point3f> > objectpoints;
    vector<cv::Point3f> world_corners;

    for (int h=0; h<6; h++)
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

        for(int l = 0; l< c.size(); l++) //for each chessboard corner
        {
            vector<Point2f> cam_points;
            vector<Point2f> proj_points;
            gem_overgeslagen = 0;
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
                + CV_CALIB_FIX_PRINCIPAL_POINT
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
                                                CV_CALIB_FIX_INTRINSIC+
                                                CV_CALIB_FIX_ASPECT_RATIO +
                                                /*CV_CALIB_ZERO_TANGENT_DIST +*/
                                                CV_CALIB_SAME_FOCAL_LENGTH +
                                                CV_CALIB_RATIONAL_MODEL +
                                                CV_CALIB_FIX_K3 /*+ CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5,
                                                TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5)*/);
    cout<<"Stereo RMS error: "<<stereo_error<<endl;


    return true;
}

///Arguments sequence: value for b, value for m, threshold, projector width, projector height
int main(int argc, char *argv[])
{
    if(argc<6)
    {
        cerr<<"not enough arguments"<<endl;
        return -1;
    }


    bool flag = true;
    b = atof(argv[1]);
    cout<<b<<endl;
    m = atof(argv[2]);
    thresh = atof(argv[3]);
    projector_width = atoi(argv[4]);
    projector_height = atoi(argv[5]);
    string dir = "./picture/";
    vector<string> files;
    bool gelukt_f = false;
    bool gelukt_d = false;
    bool gelukt_c = false;
    vector<Decoder> dec;
    vector<vector<Point2f> > corners;

    ///Count the number of series
    getdir(dir, files);
    for(uint i=0; i<files.size(); i++)
    {
        aantalseries = i;
    }
    ///Remove ".." and "." directoy from the count
    aantalseries-=1;
    cout<<"aantal series: "<<aantalseries<<endl;
    int vertical_patterns=0;

    ///Get dimensions
    while(pow(2, vertical_patterns) < projector_width)
    {
        vertical_patterns++;
    }
    NOP_v = vertical_patterns;

    ///Do the same for the horizontal patterns
    int horizontal_patterns=0;
    while(pow(2, horizontal_patterns) < projector_height)
    {
        horizontal_patterns++;
    }
    NOP_h = horizontal_patterns;

    cout<<"number of vertical patterns: "<<NOP_v<<"\n"
          "number of horizontal patterns: "<<NOP_h<<endl;

    ///Verwijderen wanneer op alle sets gewerkt moet worden:
    //aantalseries=1;

    while(flag)
    {
        cout<<"Choose your option:\n"
            " g = get calibration files\n"
            " f = find chessboard corners for each serie\n"
            " d = decode\n"
            " c = calibrate\n"
            " q = quit program"<<endl;
        char keuze;
        cin >> keuze;

        if(keuze == 'g')
        {
            ostringstream conv;
            conv << aantalseries;
            string path = "./picture/serie"+conv.str();
            mkdir(path.c_str(), 0700);
            bool gelukt = get_calib_images(300, aantalseries);
            if(gelukt)
                aantalseries++;
        }

        else if(keuze == 'f')
        {
            vector<vector<Point2f> > chessboardcorners(aantalseries);
            gelukt_f = findcorners(chessboardcorners, aantalseries);
            if(gelukt_f)
                corners = chessboardcorners;
        }

        else if(keuze == 'd')
        {
            bool draw = true;
            gelukt_d = decode_all(aantalseries, dec, draw);
        }

        else if(keuze == 'c')
        {
            if(gelukt_f && gelukt_d)
            {
                gelukt_c = calibrate(dec, corners, aantalseries);
            }
            else
            {
                cout<<"To calibrate, first find corners and decode"<<endl;
                vector<vector<Point2f> > chessboardcorners(aantalseries);
                gelukt_f = findcorners(chessboardcorners, aantalseries);
                if(gelukt_f)
                    corners = chessboardcorners;
                bool draw = false;
                gelukt_d = decode_all(aantalseries, dec, draw);
                gelukt_c = calibrate(dec, corners, aantalseries);

            }
        }

        else if(keuze == 'q')
        {
            cout<<"shutting down"<<endl;
            flag = false;
        }

        else
            cout<<"not a valid choice, choose again"<<endl;
    }
}
