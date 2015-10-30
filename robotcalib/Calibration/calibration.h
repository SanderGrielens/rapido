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

#include "VimbaCPP/Include/VimbaCPP.h"
#include "Common/StreamSystemInfo.h"
#include "Common/ErrorCodeToMessage.h"
#include "VimbaCPP/Include/VimbaSystem.h"

using namespace std;
using namespace cv;

///Structured light:
vector<Mat> generate_pattern(int NOP_v, int NOP_h, int projector_width, int projector_height);
bool get_images(int delay, int serie, int width, int height);

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



int getdir (string dir, vector<string> &files);

#endif // CALIBRATION_H_INCLUDED
