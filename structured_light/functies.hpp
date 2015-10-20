#ifndef FUNCTIES_HPP_INCLUDE
#define FUNCTIES_HPP_INCLUDED


#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <algorithm>
#include <iterator>
#include <bitset>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <sstream>
#include <time.h>
#include <limits>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <sys/stat.h>
#include <opencv2/videoio/videoio.hpp>
#include "VimbaCPP/Include/VimbaCPP.h"
#include "Common/StreamSystemInfo.h"
#include "Common/ErrorCodeToMessage.h"
#include "VimbaCPP/Include/VimbaSystem.h"

using namespace std;
using namespace cv;

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

///SL SPECIFIC FUNCTIONS AND STRUCTS
struct Decoder
{
    vector<Mat> minimum;
    vector<Mat> maximum;
    vector<Mat> Ld;
    vector<Mat> Lg;
    vector<Mat> pattern_image;
} ;

int getdir (string dir, vector<string> &files);

///CALIBRATION SPECIFIC FUNCTIONS
int Calibrate_Camera();

///STANDARD FUNCTIONS
struct Point4f {
  float x;
  float y;
  float z;
  float w;
} ;

Mat inlezen(String Im);                          // afbeelding inlezen a.d.h.v. pad
Mat inlezenGrijs(String Im);                     // afbeelding inlezen in grijswaarden a.d.h.v. pad

void opslaan(Mat image, String naam);            // afbeelding opslaan a.d.h.v. afbeelding en pad

void tonen(Mat image, String naam);              // afbeelding tonen op scherm a.d.h.v. afbeelding en naam venster
void evenTonen(Mat image, String naam);          // afbeelding tonen op scherm en terug verwijderen na ANYKEY
void snelTonen(Mat image);                       // afbeelding tonen op scherm a.d.h.v. afbeelding
void rbgwaardenTonen (Mat image);

void absoluutSchalen (Mat image, double breedte, double hoogte); 			// absoluut schalen van de afbeelding
void relatiefSchalen (Mat image, double breedteSchaling, double hoogteSchaling);	// relatief t.o.v. afbeelding schalen


/* grijswaarden */
Mat grijswaardenNiveaus (Mat image, int level);  // grijswaardeniveau's aanpassen a.d.h.v.afbeelding en gewenst aantal niveau's
void contrastStretchen (Mat image);   		 // contrast van een afbeelding uitrekken van 0 tot 255


/* detecties	*/
Mat kleurenrangeDetectie (Mat image, int min, int max, int keep); 	// kleurenrange detecteren a.d.h.v. minimale en maximale waarde
void cirkelsDetectie (Mat image, int min, int max);			// cirkels detecteren a.d.h.v. minimale en maximale grootte


/* pepper & salt */
void erosie(Mat image);				// erosie van een afbeelding (salt wegwerken)
void dilatie (Mat image);			// dilatie van een afbeelding (pepper wegwerken)
void opening (Mat image);			// opening van een afbeelding
void closing (Mat image);			// closing van een afbeelding


#endif // FUNCTIES_HPP_INCLUDED
