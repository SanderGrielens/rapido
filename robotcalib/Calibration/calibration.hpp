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

#include <ueye.h>

#include <pcl/io/ensenso_grabber.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include "VimbaCPP/Include/VimbaCPP.h"
#include "Common/StreamSystemInfo.h"
#include "Common/ErrorCodeToMessage.h"
#include "VimbaCPP/Include/VimbaSystem.h"

using namespace std;
using namespace cv;

///Ensenso:
void get_en_image();
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

Decoder init_decoder();
vector<Mat> generate_pattern(int NOP_v, int NOP_h, int projector_width, int projector_height);
bool get_sl_images(int delay, int serie, int width, int height);
int getdir (string dir, vector<string> &files);
bool findcorners(vector<vector<Point2f> > &chessboardcorners, int aantalseries, int width, int height);
int check_bit(float value1, float value2, float Ld, float Lg, float m);
void calculate_light_components(Decoder &d, vector<Mat> beelden, int dir, float b);
void get_pattern_image(Decoder &d, vector<Mat> beelden, int dir, float m, float thresh);
void colorize_pattern(Decoder &d, vector<Mat> &image, int dir, int projector_width, int projector_height);
bool decode(int serienummer, Decoder &d, bool draw, float b, float m, float thresh, int p_w, int p_h);
bool decode_all(int aantalseries, vector<Decoder> &dec, bool draw, float b, float m, float thresh, int projector_width, int projector_height);
bool calibrate_sl(vector<Decoder> dec, vector<vector<Point2f> > corners, int aantalseries, int projector_width, int projector_height);



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
