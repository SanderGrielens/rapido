/* ************************************************************ */
/* Van Dessel Shana - Beeldverwerking Labo                      */
/* FUNCTIES.CPP                                                 */
/* ************************************************************ */

// AFBEELDINGEN INLEZEN
#include "functies.hpp"


int getdir (string dir, vector<string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL)
    {
        cout << "Error(" << errno << ") opening " << dir << endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL)
    {
        files.push_back(string(dirp->d_name));
    }
    closedir(dp);
    return 0;
}

double distance_Point_Line(Scalar lijn, Point2f punt)
{
    double teller = abs(lijn[0]*punt.x + lijn[1]*punt.y + lijn[2]);
    double noemer = sqrt(pow(lijn[0], 2) + pow(lijn[1], 2));
    return teller/noemer;
}

double distance_to_Line(cv::Point line_start, cv::Point line_end, cv::Point point)
{
    double normalLength = hypot(line_end.x - line_start.x, line_end.y - line_start.y);
    double distance = (double)((point.x - line_start.x) * (line_end.y - line_start.y) - (point.y - line_start.y) * (line_end.x - line_start.x)) / normalLength;
    return abs(distance);
}

double Slope(int x0, int y0, int x1, int y1)
{
    return (double)(y1-y0)/(x1-x0);
}

void fullLine(cv::Mat *img, cv::Point a, cv::Point b, cv::Scalar color)
{
    double slope = Slope(a.x, a.y, b.x, b.y);

    Point p(0,0), q(img->cols,img->rows);

    p.y = -(a.x - p.x) * slope + a.y;
    q.y = -(b.x - q.x) * slope + b.y;

    line(*img,p,q,color,1,8,0);
}


Mat inlezen(String Im)                          // afbeelding inlezen a.d.h.v. pad
{
    Mat image;
    image = imread(Im);

    if(!image.data )
        cout <<  "Could not open or find the image" << endl ;

    return image;
}

Mat inlezenGrijs(String Im)                    // grijswaarden afbeelding inlezen a.d.h.v. pad
{
    Mat image;
    image = imread(Im,0);

    if(!image.data )
        cout <<  "Could not open or find the image" << endl ;

    return image;
}

// AFBEELDINGEN OPSLAAN

void opslaan(Mat image, String naam)            // afbeelding opslaan a.d.h.v. afbeelding en pad
{
    imwrite(naam, image);
}

// AFBEELDINGEN TONEN

void tonen(Mat image, String naam)              // afbeelding tonen op scherm a.d.h.v. afbeelding en naam venster
{
    namedWindow( naam, WINDOW_NORMAL );
    resizeWindow(naam, 1200,900);
    imshow( naam, image );
    waitKey(0);
}

void evenTonen(Mat image, String naam)          // afbeelding tonen op scherm a.d.h.v. afbeelding en naam venster
{
    // automatisch terug verwijderen
    namedWindow( naam, WINDOW_AUTOSIZE );
    imshow( naam, image );
    waitKey(0);
    destroyWindow(naam);
}

void snelTonen(Mat image)                       // afbeelding tonen op scherm a.d.h.v. afbeelding
{
    // automatisch terug verwijderen
    namedWindow( "Afbeelding", WINDOW_AUTOSIZE );
    imshow( "Afbeelding", image );
    waitKey(0);
}

void rbgwaardenTonen (Mat image)		// r-, g- en b-waarden van afbeelding afzonderlijk tonen op het scherm
{
    Mat newImage[3];

    split(image, newImage);                 	// BGR

    tonen(newImage[0], "blauw");
    tonen(newImage[1], "groen");
    tonen(newImage[2], "rood");
}

// AFBEELDINGEN SCHALEN

void absoluutSchalen (Mat image, double breedte, double hoogte) // absoluut schalen van de afbeelding
{
    if (breedte < 1)
        breedte = 1;

    if (breedte > 1000)
        breedte = 1000;

    if (hoogte < 1)
        hoogte = 1;

    if (hoogte > 1000)
        hoogte = 1000;

    Size size(breedte, hoogte);
    resize(image, image, size);
}

void relatiefSchalen (Mat image, double breedteSchaling, double hoogteSchaling)	// relatief t.o.v. afbeelding schalen
{
    if (breedteSchaling<0)
        breedteSchaling = 0.01;

    if (breedteSchaling>100)
        breedteSchaling = 100;

    if (hoogteSchaling<0)
        hoogteSchaling = 0.01;

    if (hoogteSchaling>100)
        hoogteSchaling = 100;

    float hoogte = image.rows * hoogteSchaling;
    float breedte = image.cols * breedteSchaling;

    Size size(breedte, hoogte);
    resize(image, image, size);
}

Mat grijswaardenNiveaus (Mat image, int level)            // grijswaardeniveau's aanpassen (level = aantal niveau's)
{
    Mat newImage = image.clone();
    double min, max;
    int i, j;

    if (level<2)
        level = 2;

    if (level>255)
        level = 255;

    for(i=0; i<image.rows; i++)
    {
        for(j=0; j<image.cols; j++)
        {
            newImage.at<uchar>(i,j) = newImage.at<uchar>(i,j)/ (256/level);
        }
    }

    minMaxIdx(newImage, &min, &max);        // min en max waarde afbeelding
    newImage = newImage - (int) min;        // min -> zwart 0
    newImage = newImage * (255/max);        // max -> wit 255

    return newImage;
}

void contrastStretchen (Mat image)          		// contrast stretchen (grijswaarden uitrekken van 0 tot 255)
{
    double min, max;

    minMaxIdx(image, &min, &max);       // min en max waarde afbeelding
    image = image - (int) min;        	// min -> zwart 0
    image = image * (255/(max-min));  	// max -> wit 255
}

Mat kleurenrangeDetectie (Mat image, int min, int max, int keep)	// range aan kleuren detecteren in een afbeelding met minimum- en maximum kleurwaarde
{
    // en factor 'keep'  om de gewenste kleur tebehouden (1), of wit te maken (0), (rest sowieso zwart)
    int i, j;
    Mat newImage = image.clone();

    cvtColor(newImage, newImage, CV_BGR2HSV);  // hue value saturation

    for ( i=0 ; i< newImage.rows ; i++ )
    {
        for ( j=0 ; j< newImage.cols ; j++ )
        {
            if ( (newImage.at<Vec3b>(i,j)[0] < min) || (newImage.at<Vec3b>(i,j)[0] > max))
            {
                newImage.at<Vec3b>(i,j)[2] = 0;
            }

            else if ( keep == 0 )
            {
                newImage.at<Vec3b>(i,j)[2] = 1;
            }
        }
    }

    cvtColor(newImage, newImage, CV_HSV2BGR);

    return newImage;
}

void cirkelsDetectie (Mat image, int min, int max)			// cirkels detecteren in afbeelding met een minimum en maximum grootte
{
    vector<Vec3f> circles;
    Mat newImage;

    GaussianBlur( image, newImage, Size(9, 9), 2, 2 );

    HoughCircles( newImage, circles, CV_HOUGH_GRADIENT, 1, newImage.rows/8, 200, 20, min, max );  // cirkels detecteren

    for( size_t i = 0; i < circles.size(); i++ )                                        // cirkels tekenen
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle( newImage, center, 3, Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        circle( newImage, center, radius, Scalar(0,0,255), 3, 8, 0 );
    }
}

void erosie(Mat image)				// erosie van een afbeelding (salt wegwerken)
{
    int erosion_size = 2;
    int erosion_type;

    erosion_type = MORPH_RECT;
    //erosion_type = MORPH_CROSS;
    //erosion_type = MORPH_ELLIPSE;

    Mat element = getStructuringElement( erosion_type,
                                         Size( 2*erosion_size + 1, 2*erosion_size+1 ), Point( erosion_size, erosion_size ) );

    erode( image, image, element );
}

void dilatie (Mat image)			// dilatie van een afbeelding (pepper wegwerken)
{
    int dilation_size = 1;
    int dilation_type;

    dilation_type = MORPH_RECT;
    // dilation_type = MORPH_CROSS;
    // dilation_type = MORPH_ELLIPSE;

    Mat element = getStructuringElement( dilation_type,
                                         Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );

    dilate( image, image, element );
}

void opening (Mat image)			// opening van een afbeelding
{
    erosie(image);
    dilatie(image);
}

void closing (Mat image)			// closing van een afbeelding
{
    dilatie(image);
    erosie(image);
}
