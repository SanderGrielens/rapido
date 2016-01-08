#include "calibration.hpp"

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

void tonen(Mat image, String naam)              // afbeelding tonen op scherm a.d.h.v. afbeelding en naam venster
{
    namedWindow( naam, WINDOW_NORMAL );
    resizeWindow(naam, 1200,900);
    imshow( naam, image );
    waitKey(0);
}

void printmat(Mat a)
{
    cout<<a.rows<<"x"<<a.cols<<endl;

    for(int x = 0; x<a.rows; x++)
    {
        for(int y=0; y< a.cols; y++)
        {
            cout<<a.at<double>(x,y)<<" ";
        }
        cout<<endl;
    }

}
