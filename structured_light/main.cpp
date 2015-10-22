#include "functies.hpp"

using namespace std;
using namespace cv;
using namespace AVT;
using namespace VmbAPI;

int NOP_h;
int NOP_v;
float b;
float m;
float thresh;
int projector_width;
int projector_height;
int camera_width;
int camera_height;

unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer->removeShape (str);
    }
    text_id = 0;
  }
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}

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

bool get_images(int delay, int serie, string mode)
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

        string pFileName;
        ///Save camera image
        if(mode.compare("calibration") == 0)
        {
            pFileName = "./picture/serie" + conv.str() + "/frame" + convert.str()+ ".bmp";
        }
        else if(mode.compare("scan") == 0)
        {
            pFileName = "./scan/serie" + conv.str() + "/frame" + convert.str()+ ".bmp";
        }
        else
        {
            cerr<<"no mode selected"<<endl;
            return false;
        }

        AVTWriteBitmapToFile( &bitmap, pFileName.c_str() );
    }

    return true;
}

bool findcorners(vector<vector<Point2f> > &chessboardcorners, int aantalseries)
{
    Mat board;
    Size boardSize(8,6);

    for(int i = 0; i<aantalseries; i++)
    {
        cout<<"finding corners image "<<i<<endl;
        ostringstream conv;
        conv << i;

        string path = "./picture/serie" + conv.str()+"/frame_rect0.bmp";
        board = imread(path, 0);
        if(board.empty())
        {
            cout<<"No undistorted image found, using normal image"<<endl;
            path = "./picture/serie" + conv.str()+"/frame0.bmp";

            board = imread(path, 0);
            if(board.empty())
            {
                cout<<"No image found"<<i<<endl;
                continue;
            }
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

void calculate_light_components(Decoder &d, vector<Mat> beelden, int dir)
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

void get_pattern_image(Decoder &d, vector<Mat> beelden, int dir)
{
    int holder;
    int bit;
    int NOP;
    int start;
    string richting;
    float max_goed = pow(2, NOP_v+1);
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
    /// If a pixel is in a lighted area, we add 2 raised to the power of the frame. This way we get a gray code pattern for each pixel in pattern[dir].
    /// Pattern[dir] is of type 32 bit float. If you have more than 32 patterns, change the type of pattern[dir].
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
                d.pattern_image[dir].at<float>(i,j) = 2 << NOP_v+2;// 2*(pow(2, NOP_v+2));
            }
        }
    }
    //tonen(d.minimum[dir], "minimum");
    //tonen(d.maximum[dir], "maximum");
    cout<<"pattern decoded"<<endl;
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

bool decode(int serienummer, Decoder &d, bool draw, string mode)
{
    vector<Mat> beelden ;
    ostringstream serienr;
    serienr << serienummer;
    string path;
    string einde = ".bmp";
    if(mode.compare("calibration") == 0)
    {
        path = "./picture/serie" + serienr.str() + "/frame_rect";
        Mat tmp = imread(path + "2" + einde, 0);
        if(tmp.empty())
        {
            cout<<"no undistorted calibration images found. Using normal images"<<endl;
            path = "./picture/serie" + serienr.str() + "/frame";
        }
    }
    else if(mode.compare("scan") == 0)
    {
        path = "./scan/serie" + serienr.str() + "/frame_rect";
        Mat tmp = imread(path + "2" + einde, 0);
        if(tmp.empty())
        {
            cout<<"No undistorted scan images found. Using normal images"<<endl;
            path = "./scan/serie" + serienr.str() + "/frame";
        }
    }
    else
    {
        cerr<<"No mode selected"<<endl;
        return false;
    }



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

    d = init_decoder();

    Mat newmat2 = Mat::zeros(camera_height, camera_width, CV_8UC3);
    vector<Mat> image;
    image.push_back(newmat2.clone());
    image.push_back(newmat2.clone());

    ///Get vertical
    cout<<"Vertical: "<<endl;
    int vertical = 1;
    calculate_light_components(d, beelden, vertical);
    get_pattern_image(d, beelden, vertical);


    ///Get Horizontal
    cout<<"Horizontal: "<<endl;
    int horizontal = 0;
    calculate_light_components(d, beelden, horizontal);
    get_pattern_image(d, beelden, horizontal);

    ///Draw patterns?
    if(draw)
    {
        colorize_pattern(d, image, horizontal);
        colorize_pattern(d, image, vertical);
    }
    return true;
}

bool decode_all(int aantalseries, vector<Decoder> &dec, bool draw, string mode)
{
    bool gelukt;
    for(int i=0; i< aantalseries; i++)
    {
        Decoder d;
        cout<<"decode serie "<<i<<endl;
        gelukt = decode(i, d, draw, mode);
        if(!gelukt)
            return false;

        dec.push_back(d);
        cout<<endl;
    }

    return true;
}

bool calibrate(vector<Decoder> dec, vector<vector<Point2f> > corners, int aantalseries)
{
    clock_t time1 = clock();
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
    clock_t time2 = clock();
    cout<<"tijd calib: "<< (float)(time2-time1)/CLOCKS_PER_SEC<<endl;
    ///Save calibration:

    FileStorage fs("camera_projector.xml", FileStorage::WRITE);
    fs << "fundamental" << F;
    fs << "rotation" << R;
    fs << "translation" << T;
    fs << "essential" << E;
    fs << "camera_matrix" << cam_mat;
    fs << "projector_matrix" << proj_mat;
    fs << "distortion_camera" << cam_dist;
    fs.release();

    return true;
}

vector<Point4f> naarPoint4f(Mat src)
{
    vector<Point4f> resultaat;
    Point4f punt;
    assert(src.rows == 4 && src.cols>0);

    for(int i=0; i<src.cols; i++)
    {
        punt.x = src.at<float>(0,i) / src.at<float>(3,i);
        punt.y = src.at<float>(1,i) / src.at<float>(3,i);
        punt.z = src.at<float>(2,i) / src.at<float>(3,i);
        punt.w = src.at<float>(3,i);
        resultaat.push_back(punt);
    }
    return resultaat;
}

vector<Visualizer> calculate3DPoints_all(string mode, int aantalseries)
{
    bool draw = false;
    vector<Decoder> dec;
    vector<Visualizer> viz;

    decode_all(aantalseries, dec, draw, "scan");


    for(int k = 0; k< aantalseries; k++)
    {
        Decoder d = dec[k];
        Visualizer v;
        vector<Point2f> cam_points;
        vector<Point2f> proj_points;
        for(int x = 0; x<camera_width; x++)
        {
            for(int y = 0; y<camera_height; y++)
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

        ///Building 3D point cloud
        Mat cameraMatrix;
        Mat projMatrix;
        Mat rotMat;
        Mat transMat;

        ///Get intrinsic and extrinsic camera parameters
        FileStorage fs("camera_projector.xml", FileStorage::READ);

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
                    //cout<<" "<<projmat1.at<double>(i,j);
                }
                else
                {
                    projmat1.at<double>(i,j)=0;
                    //cout<<" "<<projmat1.at<double>(i,j);
                }
            }
            //cout   <<endl;
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

        Mat P0, P1;

        Mat driedpunten;

        P0 = cameraMatrix * projmat1;
        P1 = projMatrix * projmat2;

        Mat cam, proj;
        cam = Mat(cam_points);
        proj = Mat(proj_points);

        clock_t time1 = clock();

        triangulatePoints(P0, P1, cam, proj, driedpunten);
        clock_t time2 = clock();
        cout<<"tijd trianguleren"<<(float)(time2-time1)/CLOCKS_PER_SEC<<endl;
        v.pointcloud.push_back(driedpunten);
        v.cam_points = cam_points;
        viz.push_back(v);
    }

    ///Visualize 3D point Cloud as 2D image
    for(int k = 0; k< aantalseries; k++)
    {
        Mat driepunten = viz[k].pointcloud;
        vector<Point4f> tussen;
        tussen = naarPoint4f(driepunten);

        Mat img = Mat(camera_height, camera_width, CV_32FC1);
        float ma = 0;
        float mi = 999999;
        float afstand;
        vector<float> afstandjes;
        for(int i = 0; i< tussen.size(); i++)
        {
            afstand = sqrt(pow(tussen[i].x, 2) + pow(tussen[i].y, 2) + pow(tussen[i].z, 2));
            if(afstand<1500 && afstand > 500)
            {
                if(afstand>ma)
                    ma = afstand;

                if(afstand < mi)
                    mi = afstand;
            }
            afstandjes.push_back(afstand);
        }

        for(int i = 0; i< tussen.size(); i++)
        {
            img.at<float>(viz[k].cam_points[i].y, viz[k].cam_points[i].x) = ((afstandjes[i] - mi) / (ma - mi) )*255;
        }
        Mat img_u;
        img.convertTo(img_u, CV_8UC1);
        tonen(img_u, "bla");

        vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION );
        //Kies 0 om geen compressie door te voeren
        compression_params.push_back(0);
        try {
            imwrite("3Dto2Dmapping.png", img_u, compression_params);
        }
        catch (int runtime_error){
            fprintf(stderr, "Exception converting image to JPPEG format: %s\n");
    //        return 1;
        }
    }
    return viz;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> visualize3Dpoints(vector<Visualizer> visual)
{
    Visualizer vis = visual[0];
    double X,Y,Z;
    char pr=100, pg=100, pb=100;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);//(new pcl::pointcloud<pcl::pointXYZ>);

    for(int i=0;i<vis.pointcloud.cols;i++)
    {
        //std::cout<<i<<endl;
        X = vis.pointcloud.at<float>(0,i) / vis.pointcloud.at<float>(3,i);
        Y = vis.pointcloud.at<float>(1,i) / vis.pointcloud.at<float>(3,i);
        Z = vis.pointcloud.at<float>(2,i) / vis.pointcloud.at<float>(3,i);

        pcl::PointXYZ point;
        point.x = X;
        point.y = Y;
        point.z = Z;

        point_cloud_ptr -> points.push_back(point);
    }


    point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;

    pcl::io::savePCDFileASCII ("test_pcd_rect.pcd", *point_cloud_ptr);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (point_cloud_ptr, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
    viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

    viewer->setCameraPosition(0, 0, 200, 0,0,0, 0,0,1);

    return viewer;
}

void undst(int calib_serie, int scan_serie)
{
    FileStorage fs("camera_projector.xml", FileStorage::READ);
    Mat cam;
    Mat dist;
    fs["camera_matrix"] >> cam;
    fs["distortion_camera"] >> dist;

    if(cam.empty() || dist.empty())
    {
        cout<<"No camera matrix or distortion coefficient found. Please calibrate before undistorting images"<<endl;
    }


    for(int j = 0; j< calib_serie; j++)
    {
        ostringstream serienr;
        serienr << j;
        Mat tmp;
        string path = "./picture/serie" + serienr.str() + "/frame_rect";
        string einde = ".bmp";
        tmp = imread(path + "2" + einde, 0);
        if(!tmp.empty())
        {
            cout<<"Calibration serie "<< j <<" has already been undistorted"<<endl;
            continue;
        }
        else
            path = "./picture/serie" + serienr.str() + "/frame";

        for(int i=0; i<(NOP_v+NOP_h)*2; i++)
        {
            ostringstream beeldnr;
            beeldnr << i;
            Mat u;
            Mat newmat_i = imread(path + beeldnr.str() + einde,0);
            if(newmat_i.empty())
                cout<<"no image"<< i << endl;

            undistort(newmat_i, u, cam, dist);

            //tonen(u, "undistorted");

            vector<int> compression_params;
            compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION );
            //Kies 0 om geen compressie door te voeren
            compression_params.push_back(0);

            try {
                imwrite(path +"_rect"+ beeldnr.str()+ einde, u, compression_params);
            }
            catch (int runtime_error){
                fprintf(stderr, "Exception converting image to JPPEG format: %s\n");
                return;
            }
        }
    }

    for(int j = 0; j<scan_serie; j++)
    {
        ostringstream serienr;
        serienr << j;
        Mat tmp;
        string path = "./scan/serie" + serienr.str() + "/frame_rect";
        string einde = ".bmp";
        tmp = imread(path + "2" + einde, 0);
        if(!tmp.empty())
        {
            cout<<"Scan serie "<< j <<" has already been undistorted"<<endl;
            continue;
        }
        else
            path = "./scan/serie" + serienr.str() + "/frame";

        for(int i=0; i<(NOP_v+NOP_h)*2; i++)
        {
            ostringstream beeldnr;
            beeldnr << i;
            Mat u;
            Mat newmat_i = imread(path + beeldnr.str() + einde,0);
            if(newmat_i.empty())
                cout<<"no image"<< i << endl;

            undistort(newmat_i, u, cam, dist);

            //tonen(u, "undistorted");

            vector<int> compression_params;
            compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION );
            //Kies 0 om geen compressie door te voeren
            compression_params.push_back(0);

            try {
                imwrite(path +"_rect"+ beeldnr.str()+ einde, u, compression_params);
            }
            catch (int runtime_error){
                fprintf(stderr, "Exception converting image to JPPEG format: %s\n");
                return;
            }
        }
    }
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
    m = atof(argv[2]);
    thresh = atof(argv[3]);
    projector_width = atoi(argv[4]);
    projector_height = atoi(argv[5]);
    string dir_calib = "./picture/";
    string dir_scan = "./scan/";

    vector<string> files_calib, files_scan;
    bool gelukt_f = false;
    bool gelukt_d = false;
    bool gelukt_c = false;
    vector<Decoder> dec;
    vector<vector<Point2f> > corners;
    int calib_series, scan_series;
    vector<Visualizer> visual;

    ///Count the number of calibration series
    getdir(dir_calib, files_calib);
    for(uint i=0; i<files_calib.size(); i++)
    {
        calib_series = i;
    }
    ///Remove ".." and "." directoy from the count
    calib_series-=1;

    cout<<"aantal calibratie series: "<<calib_series<<endl;

    ///Count the number of scan series
    getdir(dir_scan, files_scan);
    for(uint i=0; i<files_scan.size(); i++)
    {
        scan_series = i;
    }
    ///Remove ".." and "." directoy from the count
    scan_series-=1;

    cout<<"aantal calibratie series: "<<scan_series<<endl;
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

    while(flag)
    {
        cout<<"Choose your option:\n"
            " g = get calibration files\n"
            " u = undistort all images\n"
            " f = find chessboard corners for each serie\n"
            " d = decode calibration images\n"
            " c = calibrate\n"
            " r = calculate 3D points \n"
            " t = visualize 3D points\n"
            " q = quit program"<<endl;
        char keuze;
        cin >> keuze;


        if(keuze == 'g')
        {
            ostringstream conv;
            conv << calib_series;
            string path = "./picture/serie"+conv.str();
            mkdir(path.c_str(), 0700);
            bool gelukt = get_images(300, calib_series, "calibration");
            if(gelukt)
                calib_series++;
        }
        else if(keuze == 'u')
        {
            undst(calib_series, scan_series);
        }
        else if(keuze == 'f')
        {
            vector<vector<Point2f> > chessboardcorners(calib_series);
            gelukt_f = findcorners(chessboardcorners, calib_series);
            if(gelukt_f)
                corners = chessboardcorners;
        }

        else if(keuze == 'd')
        {
            bool draw = false;
            clock_t time1 = clock();
            gelukt_d = decode_all(calib_series, dec, draw, "calibration");
            clock_t time2 = clock();
            cout<<"tijd decode: "<< (float)(time2-time1)/CLOCKS_PER_SEC<<endl;
        }

        else if(keuze == 'c')
        {
            clock_t time1 = clock();
            if(!gelukt_f)
            {
                clock_t time2 = clock();
                vector<vector<Point2f> > chessboardcorners(calib_series);
                gelukt_f = findcorners(chessboardcorners, calib_series);
                if(gelukt_f)
                    corners = chessboardcorners;
                clock_t time3 = clock();
                cout<<"tijd findcorners: "<< (float)(time3-time2)/CLOCKS_PER_SEC<<endl;
            }
            if(!gelukt_d)
            {
                clock_t time4 = clock();
                bool draw = false;
                gelukt_d = decode_all(calib_series, dec, draw, "calibration");
                clock_t time5 = clock();
                cout<<"tijd decode: "<< (float)(time5-time4)/CLOCKS_PER_SEC<<endl;
            }

            if(gelukt_f && gelukt_d)
            {
                gelukt_c = calibrate(dec, corners, calib_series);
            }
            clock_t time6 = clock();
            cout<<"tijd calibrate: "<< (float)(time6-time1)/CLOCKS_PER_SEC<<endl;
        }

        else if(keuze == 'r')
        {
            clock_t time1 = clock();
            visual = calculate3DPoints_all("scan", scan_series);
            clock_t time2 = clock();
            cout<<"tijd teken: "<< (float)(time2-time1)/CLOCKS_PER_SEC<<endl;
        }

        else if(keuze == 't')
        {
            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
            viewer = visualize3Dpoints(visual);
            cerr<<"viewer"<<endl;
            while (!viewer->wasStopped ())
            {
                viewer->spinOnce (100);
                boost::this_thread::sleep (boost::posix_time::microseconds (100000));
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
