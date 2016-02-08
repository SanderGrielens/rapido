#ifndef CALIBRATION_H_INCLUDED
#define CALIBRATION_H_INCLUDED

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <sys/stat.h>
#include <limits>
#include <omp.h>
#include <sstream>
#include <iomanip>

#include <ueye.h>

#include <pcl/io/ensenso_grabber.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>

#include "VimbaCPP/Include/VimbaCPP.h"
#include "Common/StreamSystemInfo.h"
#include "Common/ErrorCodeToMessage.h"
#include "VimbaCPP/Include/VimbaSystem.h"

#include "flycapture/FlyCapture2.h"
#include "flycapture/stdafx.h"

#include <boost/timer/timer.hpp>
#include <cmath>

#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace boost;
using namespace FlyCapture2;

///Normal camera
void calibrate_camera();
Mat calculate3D_single_camera();


///Ensenso:
void get_en_image(pcl::PointCloud<pcl::PointXYZ>&);
pcl::PointCloud<pcl::PointXYZ> get_en_cloud();


///Structured light:
struct Decoder
{
    vector<Mat> minimum;
    vector<Mat> maximum;
    vector<Mat> Ld;
    vector<Mat> Lg;
    vector<Mat> pattern_image;
} ;

struct Paar
{
    vector<Point2d> camera;
    Point2d projector;
} ;

struct Visualizer
{
    Mat pointcloud;
    vector<Point2d> cam_points;
} ;

Decoder init_decoder();
vector<Mat> generate_pattern(int NOP_v, int NOP_h, int projector_width, int projector_height);
bool get_sl_images(int delay, string path, int serie, int width, int height);
bool get_vimba(int delay, string path, int serie, int width, int height);
bool get_pointgrey(int delay, string path, int serie, int width, int height);
int getdir (string dir, vector<string> &files);
bool findcorners(vector<vector<Point2f> > &chessboardcorners, string path, int aantalseries, int width, int height);
int check_bit(float value1, float value2, float Ld, float Lg, float m);
void calculate_light_components(Decoder &d, vector<Mat> beelden, int dir, float b);
void get_pattern_image(Decoder &d, vector<Mat> beelden, int dir, float m, float thresh);
void colorize_pattern(Decoder &d, vector<Mat> &image, int dir, int projector_width, int projector_height);
bool decode(int serienummer, Decoder &d, bool draw, string path, float b, float m, float thresh, int p_w, int p_h);
bool decode_all(int aantalseries, vector<Decoder> &dec, bool draw, string path, float b, float m, float thresh, int projector_width, int projector_height);
bool calibrate_sl(vector<Decoder> dec, vector<vector<Point2f> > corners, int aantalseries, int projector_width, int projector_height);
Mat calculate3DPoints_all(string path, int aantalseries, float b, float m, float thresh, int projector_width, int projector_height);
Mat calculate3DPoints(vector<Point2f> &chessboardcorners, Decoder d);
Mat getRobotPoints(int height, int width);
Mat calculateTransMat(Mat origin, Mat dest);
Mat convert(Mat origin);
void getRmsError(Mat transformed, Mat dest);
bool calibrate_sl_r(string path, float b, float m, float thresh, int projector_width, int projector_height);


///MultiFlash:
void grabMultiflashImages();
void calibrateMultiflashCamera();
void calculateEdgeMap();
void calculateEdgeMapRGB();
void grabMultiflashCalibImages(int number);
Mat grabSingleImage(String a, HIDS hCam);




    ///VIMBA SDK FUNCTIONS AND STRUCTS
    typedef enum
    {
        ColorCodeMono8  = 1,
        ColorCodeMono16 = 2,
        ColorCodeBGR24  = 4,
        ColorCodeRGB24  = 8
    } ColorCode;

    typedef struct
    {
        void*           buffer;
        unsigned long   bufferSize;
        unsigned long   width;
        unsigned long   height;
        ColorCode       colorCode;
    } AVTBitmap;

    unsigned char AVTCreateBitmap( AVTBitmap * const pBitmap, const void* pBuffer );
    unsigned char AVTReleaseBitmap( AVTBitmap * const pBitmap );
    unsigned char AVTWriteBitmapToFile( AVTBitmap const * const pBitmap, char const * const pFileName );

#endif // CALIBRATION_H_INCLUDED

///General
void tonen(Mat image, String naam);

void printmat(Mat a);
void printmat(Mat a, String b);
void printmat(vector<Mat> a, String b);

void save(Mat origin);



