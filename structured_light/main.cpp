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
int projector_width;
int projector_height;

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
    }

    return true;
}

bool findcorners(vector<vector<Point2f> > &chessboardcorners, int aantalseries)
{
    Mat board;
    Size boardSize(6,8);
    for(int i = 0; i<aantalseries; i++)
    {
        cout<<"nummer "<<i<<endl;
        ostringstream conv;
        conv << i;
        string path = "./picture/serie" + conv.str() + "/frame0.bmp";
        board = imread(path, 0);
        if(board.empty())
            cout<<"leeg"<<endl;

        bool found1_1 = findChessboardCorners(board, boardSize, chessboardcorners[i],
                                              CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_NORMALIZE_IMAGE + CV_CALIB_CB_FILTER_QUADS);

        if(!found1_1)
        {
            std::cerr << "Checkboard 1_"<<i<<" corners not found!" << std::endl;
            return false;
        }
        cornerSubPix(board, chessboardcorners[i], Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5));
        cvtColor(board, board, CV_GRAY2BGR);
        drawChessboardCorners( board, boardSize, chessboardcorners[i], found1_1 );
        tonen(board, "zijn ze gevonden?");
    }

    return true;
}

unsigned short check_bit(unsigned value1, unsigned value2, unsigned Ld, unsigned Lg, unsigned m)
{
    if (Ld < m)
    {
        return BIT_UNCERTAIN;
    }
    if (Ld>Lg)
    {
        return (value1>value2 ? 1 : 0);
    }
    if (value1<=Ld && value2>=Lg)
    {
        return 0;
    }
    if (value1>=Lg && value2<=Ld)
    {
        return 1;
    }
    return BIT_UNCERTAIN;
}

void calculate_light_components(Mat &Ld, Mat &Lg, vector<Mat> beelden)
{
    float minld = 1000;
    float maxld = 0;
    float minlg = 1000;
    float maxlg = 0;
    int NOP = NOP_h+NOP_v;

    for(int i =0; i< beelden[NOP].rows; i++)
    {
        for(int j =0; j<beelden[NOP].cols; j++)
        {

            int lpmax=0;
            int lpmin=1000;
            for(int k = 2; k< 6; k++)
            {
                if(beelden[NOP*2-k].at<uchar>(i,j) < lpmin)
                    lpmin = beelden[NOP*2-k].at<uchar>(i,j);
            }

            for(int k = 2; k<6; k++)
            {
                if(beelden[NOP*2-k].at<uchar>(i,j) > lpmax)
                    lpmax = beelden[NOP*2-k].at<uchar>(i,j);
            }

            Ld.at<float>(i,j) = (lpmax - lpmin) / (1 - b);
            Lg.at<float>(i,j) = 2*(lpmin - b*lpmax) / (1- pow(b,2));


            /*if(Ld.at<float>(i,j) < minld)
                minld = Ld.at<float>(i,j);
            if( Ld.at<float>(i,j) > maxld)
                maxld = Ld.at<float>(i,j);

            if(Lg.at<float>(i,j) < minlg)
                minlg = Lg.at<float>(i,j);
            if( Lg.at<float>(i,j) > maxlg)
                maxlg = Lg.at<float>(i,j);*/
        }
    }

    cout<<"direct and global light calculated"<<endl;
}

bool decode(int serienummer)
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
        Mat newmat = imread(path + beeldnr.str() + einde,0);
        if(newmat.empty())
            cout<<"no image"<<endl;

        beelden.push_back(newmat);
    }

    ///Calculate the matrices Ld and Lg of the series (Direct light and Global Light respectively)
    ///DIT IS NOG VERKEERD!
    Mat Ld(beelden[NOP_v].rows, beelden[NOP_v].cols, CV_32F);
    Mat Lg(beelden[NOP_v].rows, beelden[NOP_v].cols, CV_32F);

    calculate_light_components(Ld, Lg, beelden);

    int total_images = NOP_v+NOP_h; ///Number of images, without fully lighted, fully dark, red, green and blue.
    int total_patterns = total_images/2;
    int total_bits = total_patterns/2;
    int holder = total_bits;
    Mat pattern_image [2];
    pattern_image [0]= Mat(beelden[1].size(), CV_32FC2);
    pattern_image[1] = Mat(beelden[1].size(), CV_32FC2);
    int t=3;

    ///Go over each image pair (img and his inverse) and check for each pixel it's value.
    for(int i = 0; i<=2*(NOP_v + NOP_h) ; i+=2)
    {
        Mat img1 = beelden[i].clone();
        Mat img2 = beelden[i+1].clone();
        int channel = 0;

        if(i == 2*NOP_v )
        {
            channel = 1;
        }

        int bit = --holder;

        for(int j =0; j< img1.rows; j++)
        {
            for(int k =0; k<img1.cols; k++)
            {
                if (pattern_image[channel].at<float>(j,k)!=PIXEL_UNCERTAIN)
                {
                    unsigned val1 = img1.at<uchar>(j,k);
                    unsigned val2 = img2.at<uchar>(j,k);
                    unsigned ld = Ld.at<float>(j,k);
                    unsigned lg = Lg.at<float>(j,k);
                    unsigned short p = check_bit(val1, val2, ld, lg, m);
                    if(p == BIT_UNCERTAIN)
                        pattern_image[channel].at<float>(j,k) = PIXEL_UNCERTAIN;
                    else
                        pattern_image[channel].at<float>(j,k) += p<<bit;
                }
            }
        }

        cout<<i/2<<"th bit pattern decoded"<<endl;
    }

    Mat image(beelden[1].size(), CV_8UC3);

    float max_t = projector_width;
    float n = 4.f;
    float dt = 255.f/n;
    int set = 0;
    for(int i = 0; i< pattern_image[0].rows; i++)
    {
        for(int j = 0; j< pattern_image[0].cols; j++)
        {
            if (pattern_image[0].at<float>(i,j) > projector_width || pattern_image[0].at<float>(i,j) == PIXEL_UNCERTAIN)
            {   //invalid value: use grey
                image.at<Vec3b>(i,j) = Vec3b(128, 128, 128);
                continue;
            }

            //display
            float t = pattern_image[0].at<float>(i,j)*255.f/max_t;
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
            image.at<Vec3b>(i,j) = Vec3b(c3, c2,c1);
        }
    }

    tonen(image, "kleur");

    return true;
}

bool decode_all(int aantalseries)
{
    bool gelukt;
    for(int i=0; i< aantalseries; i++)
    {
        cout<<"decode next"<<endl;
        cout<<endl;
        gelukt = decode(i);
        if(!gelukt)
            return false;
    }

    return true;
}

int main(int argc, char *argv[])
{
    if(argc<5)
    {
        cerr<<"not enough arguments"<<endl;
        return -1;
    }


    bool flag = true;
    b = atoi(argv[1]);
    m = atoi(argv[2]);
    projector_width = atoi(argv[3]);
    projector_height = atoi(argv[4]);
    string dir = "./picture/";
    vector<string> files;

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



    while(flag)
    {
        cout<<"Choose your option:\n"
            " c = get calibration files\n"
            " f = find chessboard corners for each serie\n"
            " d = decode\n"
            " q = quit program"<<endl;
        char keuze;
        cin >> keuze;

        if(keuze == 'c')
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
            bool gelukt = findcorners(chessboardcorners, aantalseries);
        }

        else if(keuze == 'd')
            bool gelukt = decode_all(aantalseries);

        else if(keuze == 'q')
        {
            cout<<"shutting down"<<endl;
            flag = false;
        }

        else
            cout<<"not a valid choice, choose again"<<endl;
    }
}

