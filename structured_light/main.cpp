#include "functies.hpp"

using namespace std;
using namespace cv;
using namespace AVT;
using namespace VmbAPI;

int aantalseries;
float b;
float m;

const float PIXEL_UNCERTAIN = std::numeric_limits<float>::quiet_NaN();
const unsigned short BIT_UNCERTAIN = 0xffff;

bool get_calib_images(int delay, int number_of_patterns, int serie)
{
    Mat pattern;
    cout<<"serienummer: "<<serie<<endl;
    ostringstream conv;
    conv << serie;

    CameraPtr camera ;
    FramePtr frame;
    char * pCameraID   = NULL;

    VimbaSystem & system = VimbaSystem :: GetInstance ();
    if ( !VmbErrorSuccess == system.Startup () )
    {
        cout<<"system startup failed"<<endl;
    }

    for(int i = 1; i<=number_of_patterns; i++)
    {
        string Result;
        ostringstream convert;
        convert << i;
        Result = "patterns/pattern" + convert.str() + ".png";

        pattern = imread(Result, 0);
        if(pattern.empty())
            return -1;

        ///Project pattern full screen via projector
        namedWindow( "pattern", CV_WINDOW_NORMAL );
        setWindowProperty("pattern", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
        moveWindow("pattern", 0, 0);
        imshow("pattern", pattern);
        waitKey(delay);

        ///Read from camera

        VmbErrorType res = system.OpenCameraByID(pCameraID, VmbAccessModeFull, camera);

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

        camera->Close();

        VmbUint32_t nImageSize = 0;
        VmbErrorType err = frame->GetImageSize( nImageSize );
        VmbUint32_t nWidth = 0;
        err = frame->GetWidth( nWidth );
        VmbUint32_t nHeight = 0;
        err = frame->GetHeight( nHeight );
        VmbUchar_t *pImage = NULL;
        err = frame->GetImage(pImage );

        AVTBitmap bitmap;
        bitmap.colorCode = ColorCodeRGB24;
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
        string path = "./picture/serie" + conv.str() + "/frame1.bmp";
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

bool decode(int serienummer)
{
    Mat beelden[NOP+1];
    ostringstream serienr;
    serienr << serienummer;
    string path = "./picture/serie" + serienr.str() + "/frame";
    string einde = ".bmp";

    ///Reading every image in a serie
    for(int i=1; i<=NOP; i++)
    {
        ostringstream beeldnr;
        beeldnr << i;
        beelden[i] = imread(path + beeldnr.str() + einde);
        if(beelden[i].empty())
            cout<<"no image"<<endl;

        cvtColor(beelden[i], beelden[i], CV_BGR2GRAY);
    }

    Mat pattern_image = Mat(beelden[0].size(), CV_32FC2);;

    ///Calculate the matrices Ld and Lg of the series (Direct light and Global Light respectively)
    Mat Ld(beelden[NOP-2].rows, beelden[NOP-2].cols, CV_32F);
    Mat Lg(beelden[NOP-2].rows, beelden[NOP-2].cols, CV_32F);

    float minld = 1000;
    float maxld = 0;
    float minlg = 1000;
    float maxlg = 0;

    for(int i =0; i< beelden[NOP].rows; i++)
    {
        for(int j =0; j<beelden[NOP].cols; j++)
        {

            int lpmax=0;
            int lpmin=1000;
            for(int k = 2; k< 6; k++)
            {
                if(beelden[NOP-k].at<uchar>(i,j) < lpmin)
                    lpmin = beelden[NOP-k].at<uchar>(i,j);
            }

            for(int k = 2; k<6; k++)
            {
                if(beelden[NOP-k].at<uchar>(i,j) > lpmax)
                    lpmax = beelden[NOP-k].at<uchar>(i,j);
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

    int total_images = NOP - 5; ///Number of images, without fully lighted, fully dark, red, green and blue.
    int total_patterns = total_images/2;
    int total_bits = total_patterns/2;

    int holder = total_bits;

    ///Go over each image pair (img and his inverse) and check for each pixel it's value.
    for(int i = 3; i<=NOP-3 ; i+=2)
    {
        Mat img1 = beelden[i].clone();
        Mat img2 = beelden[i].clone();

        int bit = --holder;

        for(int j =0; j< img1.rows; j++)
        {
            for(int k =0; k<img1.cols; k++)
            {
                unsigned val1 = img1.at<uchar>(j,k);
                unsigned val2 = img2.at<uchar>(j,k);
                unsigned ld = Ld.at<float>(j,k);
                unsigned lg = Lg.at<float>(j,k);
                unsigned short p = check_bit(val1, val2, ld, lg, m);
                if(p == BIT_UNCERTAIN)
                    pattern_image = PIXEL_UNCERTAIN;
                else
                    pattern_image += p<<bit;
            }
        }
    }

    cout << pattern_image<<endl;

    //tonen(Ld/maxld, "Direct");
    //tonen(Lg/maxlg, "globaal");
    return true;
}

bool decode_all(int aantalseries)
{
    bool gelukt;
    for(int i=0; i< aantalseries; i++)
    {
        gelukt = decode(i);
        if(!gelukt)
            return false;
    }

    return true;
}

int main(int argc, char *argv[])
{
    if(argc<3)
    {
        cerr<<"not enough arguments"<<endl;
        return -1;
    }


    bool flag = true;
    b = atoi(argv[1]);
    m = atoi(argv[2]);
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
            bool gelukt = get_calib_images(300, NOP, aantalseries);
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

