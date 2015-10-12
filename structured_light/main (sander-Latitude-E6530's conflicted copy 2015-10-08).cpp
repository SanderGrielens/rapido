#include "functies.hpp"

using namespace std;
using namespace cv;

int aantalseries;

bool get_calib_images(int delay, int number_of_patterns, int serie)
{
    Mat pattern;
    cout<<"serienummer: "<<serie<<endl;
    ostringstream conv;
    conv << serie;

    int CLASS = 0;

    #ifdef _MSC_VER
        int CLASS = CV_CAP_DSHOW;
    #endif
    #ifdef Q_OS_MAC
        int CLASS = CV_CAP_QT;
    #endif
    #ifdef Q_OS_LINUX
        int CLASS = CV_CAP_PVAPI;
    #endif

    VideoCapture videoInput(CLASS);

    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION );
    //Kies 0 om geen compressie door te voeren
    compression_params.push_back(0);
    videoInput.open(CLASS);
    if(!videoInput.isOpened())
    {
        cerr<<"no camera opened"<<endl;
        return false;
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

        ///Read out camera image
        Mat cam_img;
        videoInput >> cam_img;
        cout<<"en schrijven maar"<<i<<endl;
        //tonen(cam_img, "ffkijk");
        ///Save recorded calibration images
        try {
            imwrite("./picture/serie" + conv.str() + "/frame" + convert.str()+ ".png", cam_img, compression_params);
        }
        catch (int runtime_error){
            fprintf(stderr, "Exception converting image to JPPEG format: %s\n");
            return 1;
        }

        waitKey(delay);
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
        string path = "./picture/serie" + conv.str() + "/frame1.png";
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

bool decode(int serienummer)
{


    return true;
}

bool decode_all()
{
    bool gelukt;
    for(int i=0; i< aantalseries;i++)
    {
        gelukt = decode(i);
        if(!gelukt)
            return false;
    }

    return true;
}

int main(int argc, char *argv[])
{

    bool flag = true;

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
            bool gelukt = get_calib_images(300, 26, aantalseries);
            if(gelukt)
                aantalseries++;
        }

        else if(keuze == 'f')
        {
            vector<vector<Point2f> > chessboardcorners(aantalseries);
            bool gelukt = findcorners(chessboardcorners, aantalseries);
        }

        else if(keuze == 'd')
            bool gelukt = decode_all();

        else if(keuze == 'q')
        {
            cout<<"shutting down"<<endl;
            flag = false;
        }

        else
            cout<<"not a valid choice, choose again"<<endl;
    }
}

