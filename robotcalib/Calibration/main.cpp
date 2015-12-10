#include "calibration.hpp"

int main()
{
    vector<Mat> calib_beelden;
    bool flag = true;
    bool gelukt_f = false;
    bool gelukt_d = false;
    bool gelukt_c = false;
    vector<vector<Point2f> > corners;
    vector<Decoder> dec;


    string dir_calib_sl = "./calib_sl/";

    vector<string> files_calib_sl;
    int calib_sl_series=0;

    ///Count the number of structured light calibration series
    getdir(dir_calib_sl, files_calib_sl);
    for(uint i=0; i<files_calib_sl.size(); i++)
    {
        calib_sl_series = i;
    }
    ///Remove ".." and "." directoy from the count
    calib_sl_series-=2;

    while(flag)
    {
        cout<<"What do you want to calibrate?\n"
          " e: Ensenso \n"
          " n: Normal camera **under construction**\n"
          " s: Structured Light\n"
          " t: TOF **under construction**\n"
          " m: Multi flash **under construction**\n"
          " l: Laser line triangulation **under construction**\n"
          " q: Quit program"<<endl;
        char keuze;
        cin >> keuze;

        if(keuze=='e')
        {
            cout<<"Press 'c' to get calibration images, or 'p' to get the pointcloud"<<endl;
            char antwoord;
            cin >> antwoord;
            pcl::PointCloud<pcl::PointXYZ> a;
            get_en_image(a);
            pcl::PLYWriter plywriter;
            pcl::io::savePCDFileBinary("./calib_en/Ensenso.pcd", a);
            plywriter.write("./calib_en/Ensenso.ply", a, false);
            /*if(antwoord == 'c')
            {
                get_en_image();
            }
            else if(antwoord == 'p')
            {
                pcl::PointCloud<pcl::PointXYZ> a = get_en_cloud();
                pcl::PLYWriter plywriter;
                pcl::io::savePCDFileBinary("./calib_en/Ensenso.pcd", a);
                plywriter.write("./calib_en/Ensenso.ply", a, false);
            }*/
        }
        else if(keuze=='n')
        {

        }
        else if(keuze=='s')
        {
            bool vlag = false;
            while(!vlag)
            {
                int p_w = 1280;
                int p_h = 800;
                float b = 0.5;
                float m = 5;
                float thresh = 15;
                /*cout<<"Please give the projector resolution. First the width, then the height:"<<endl;
                cin >> p_w;
                cin >> p_h;*/

                cout<<"To calibrate the sensor, press s\n"
                        "To calibrate the robot with reference to the sensor, press r\n"
                        "To quit, press q"<<endl;
                char antwoord;
                cin >> antwoord;

                if(antwoord == 's')
                {
                    cout<<"The resolution you gave is: "<<p_w<<"x"<<p_h<<endl;
                    cout<<"Do you wish to make new calibration images? y/n"<<endl;
                    char antwoord;
                    cin >> antwoord;
                    while(antwoord == 'y')
                    {
                        ostringstream conv;
                        conv << calib_sl_series;
                        string path = "./calib_sl/serie"+conv.str();
                        mkdir(path.c_str(), 0700);
                        bool gelukt = get_sl_images(300, path, calib_sl_series, p_w, p_h);
                        if(gelukt)
                            calib_sl_series++;
                        else
                            cout<<"failed to get serie: "<<calib_sl_series<<endl;
                        cout<<"Another?"<<endl;
                        cin >> antwoord;
                    }

                    if(!gelukt_f)
                    {
                        string path = "./calib_sl/serie";
                        vector<vector<Point2f> > chessboardcorners(calib_sl_series);
                        gelukt_f = findcorners(chessboardcorners, path, calib_sl_series, p_w, p_h);
                        if(gelukt_f)
                            corners = chessboardcorners;
                    }
                    if(!gelukt_d)
                    {
                        string path = "./calib_sl/serie";
                        bool draw = false;
                        gelukt_d = decode_all(calib_sl_series, dec, draw, path, b, m, thresh, p_w, p_h);
                    }

                    if(gelukt_f && gelukt_d)
                    {
                        gelukt_c = calibrate_sl(dec, corners, calib_sl_series, p_w, p_h);
                        if(!gelukt_c)
                            cout<<"Structured Light setup failed to calibrate."<<endl;
                    }
                }
                else if(antwoord == 'r')
                {
                    string path = "./robot_sl";
                    //bool gelukt = get_sl_images(300, path, 0, p_w, p_h);
                    //clock_t time1 = clock();
                    //boost::timer::auto_cpu_timer t;
                    struct timeval tv1,tv2; struct timezone tz;
                    gettimeofday(&tv1, &tz);
                    bool gelukt_cr  = calibrate_sl_r(path, b, m, thresh, p_w, p_h);
                    gettimeofday(&tv2, &tz);
                    printf( "wall clock time (gettimeofday)  = %12.4g sec\n", (tv2.tv_sec-tv1.tv_sec) + (tv2.tv_usec-tv1.tv_usec)*1e-6 );
                    //clock_t time2 = clock();
                    //cout<<"tijd calibreren robot "<<(float)(time2-time1)/CLOCKS_PER_SEC<<endl;
                }
                else if(antwoord == 'q')
                {
                    vlag = true;
                }
            }
        }
        else if(keuze=='t')
        {

        }
        else if(keuze=='m')
        {

        }
        else if(keuze=='l')
        {

        }
        else if(keuze=='q')
        {
            cout<<"shutting down"<<endl;
            flag = false;
        }
        else
            cout<<"Not a valid choice, choose again."<<endl;
    }

    return 0;
}

