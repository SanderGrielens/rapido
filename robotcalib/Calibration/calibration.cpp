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
    resizeWindow(naam, 1200,800);
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
            if(a.at<double>(x,y) < 0.000001 && a.at<double>(x,y) > -0.000001)
                cout<<"0"<<" ";
            else
                cout<<a.at<double>(x,y)<<" ";
        }
        cout<<endl;
    }

}

void printmat(Mat a, String b)
{
    cout<<b<<endl;
    cout<<a.rows<<"x"<<a.cols<<endl;

    for(int x = 0; x<a.rows; x++)
    {
        for(int y=0; y< a.cols; y++)
        {
            if(a.at<double>(x,y) < 0.000001 && a.at<double>(x,y) > -0.000001)
                cout<<"0"<<" ";
            else
                cout<<a.at<double>(x,y)<<" ";
        }
        cout<<endl;
    }

}

void printmat(vector<Mat> a, String b)
{
    cout<<b<<endl;
    cout<<a[0].rows<<"x"<<a.size()<<endl;
    //for(int i=0; i<a.size(); i++)
    //{
        for(int x = 0; x<a[0].rows; x++)
        {
            for(int y=0; y< a.size(); y++)
            {
                if(a[y].at<double>(x,0) < 0.000001 && a[y].at<double>(x,0) > -0.000001)
                    cout<<"0"<<" ";
                else
                    cout<<a[y].at<double>(x,0)<<" ";
            }
            cout<<endl;
        }
}

