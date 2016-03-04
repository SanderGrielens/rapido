#include <iostream>
#include <math.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <opencv2/opencv.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>


using namespace std;
using namespace cv;

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, string nummer)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud" + nummer);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud" + nummer);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


void tonen(Mat image, String naam)              // afbeelding tonen op scherm a.d.h.v. afbeelding en naam venster
{
    namedWindow( naam, WINDOW_NORMAL );
    resizeWindow(naam, 1200,800);
    imshow( naam, image );
    waitKey(0);
}

int rms_error_ground_plane()
{
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_collection; //(new pcl::PointCloud<pcl::PointXYZ>);
   // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_en(new pcl::PointCloud<pcl::PointXYZ>);

    vector<string> files;

    files.push_back("en.ply");
    files.push_back("sl.ply");

    pcl::PLYReader plyreader;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    for(int k=0; k<files.size() ; k++)
    {
        ostringstream conv;
        conv << k;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        plyreader.read(files[k], *cloud);
        cout<<files[k]<<endl;
        cloud->width  = cloud->points.size();
        cloud->height = 1;
        cloud->points.resize (cloud->width * cloud->height);

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.003);

        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            return (-1);
        }

        while(abs(coefficients->values[1] ) > 0.01)
        {
            cout<<"rotate x"<<endl;
            ///Build our own rotation matrix
            Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
            float theta;
            // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
            theta = asin(coefficients->values[1]);
            //rotate around X-Axis
            transform_1 (1,1) = cos (theta);
            transform_1 (1,2) = -sin(theta);
            transform_1 (2,1) = sin (theta);
            transform_1 (2,2) = cos (theta);
            //    (row, column)

            pcl::transformPointCloud (*cloud, *cloud, transform_1);

            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);

            if (inliers->indices.size () == 0)
            {
                PCL_ERROR ("Could not estimate a planar model for the given dataset.");
                return (-1);
            }
        }

        while(abs(1.0 - coefficients->values[2] ) > 0.01) //rotate around Y-axis
        {
            cout<<"rotate y"<<endl;
            ///Build our own rotation matrix
            Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
            float theta;
            // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
            theta = (M_PI_2) - asin(coefficients->values[2]);
            //rotate around Y-Axis
            transform_1 (0,0) = cos (theta);
            transform_1 (2,0) = -sin(theta);
            transform_1 (0,2) = sin (theta);
            transform_1 (2,2) = cos (theta);
            //    (row, column)

            pcl::transformPointCloud (*cloud, *cloud, transform_1);

            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);

            if (inliers->indices.size () == 0)
            {
                PCL_ERROR ("Could not estimate a planar model for the given dataset.");
                return (-1);
            }
        }



        std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                              << coefficients->values[1] << " "
                                              << coefficients->values[2] << " "
                                              << coefficients->values[3] << std::endl;

        std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

        double hoogstez = 0;
        double hoogste2 = 0;
        double laagstez = 10000;
        double laagste2 = 10000;

        for(int i=0; i<cloud->points.size(); i++)
        {
            if(cloud->points[i].z < (-coefficients->values[3] + 0.01))
            {
                if(hoogstez<cloud->points[i].z)
                {
                    hoogste2 = hoogstez;
                    hoogstez = cloud->points[i].z;
                }
                if(laagstez>cloud->points[i].z)
                {
                    laagste2 = laagstez;
                    laagstez = cloud->points[i].z;
                }
            }

        }


        double X,Y,Z;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);//(new pcl::pointcloud<pcl::pointXYZ>);
        float n = 4.f;
        float dt = 255.f/n;
        for(int i=0;i<cloud->points.size();i++)
        {
            //std::cout<<i<<endl;
            X = cloud->points[i].x;
            Y = cloud->points[i].y;
            Z = cloud->points[i].z;

            pcl::PointXYZRGB point;
            point.x = X;
            point.y = Y;
            point.z = Z;

            float t = ((Z-laagste2)/(hoogste2-laagste2))*255;
            float c1 = 0.f, c2 = 0.f, c3 = 0.f;
            if (t<=1.f*dt)
            {   //black -> red

                float c = n*(t-0.f*dt);
                c1 = c;     //0-255
                c2 = 0.f;   //0
                c3 = 0.f;   //0
            }
            else if (t<=2.f*dt)
            {   //red -> red,green

                float c = n*(t-1.f*dt);
                c1 = 255.f; //255
                c2 = c;     //0-255
                c3 = 0.f;   //0
            }
            else if (t<=3.f*dt)
            {   //red,green -> green
                float c = n*(t-2.f*dt);
                c1 = 255.f-c;   //255-0
                c2 = 255.f;     //255
                c3 = 0.f;       //0
            }
            else if (t<=4.f*dt)
            {   //green -> blue
                float c = n*(t-3.f*dt);
                c1 = 0.f;       //0
                c2 = 255.f-c;   //255-0
                c3 = c;         //0-255
            }

            uint8_t r = c1, g = c2, b = c3;
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            point.rgb = *reinterpret_cast<float*>(&rgb);
            if(Z < (-coefficients->values[3] + 0.01))
            {
                //cout<<Z<<endl;
                point_cloud_ptr -> points.push_back(point);
            }
        }

        pcl::PLYWriter plywriter;
        plywriter.write("./recht.ply", *point_cloud_ptr, false);
        pcl::io::savePCDFileBinary("./recht" + conv.str() + ".pcd", *point_cloud_ptr);


        ///Lengte van de diagonaal berekenen
        double xmin = 0, xmax = 0, ymin = 0, ymax = 0;
        for(int i = 0; i<cloud->points.size(); i++)
        {
            if(cloud->points[i].x < xmin)
                xmin = cloud->points[i].x;
            if(cloud->points[i].x > xmax)
                xmax = cloud->points[i].x;
            if(cloud->points[i].y < ymin)
                ymin = cloud->points[i].y;
            if(cloud->points[i].y > ymax)
                ymax = cloud->points[i].y;
        }

        double diagonaal = sqrt(pow(xmax-xmin, 2) + pow(ymax - ymin, 2));
        ///Grondvlak tekenen + gemiddelde afwijking uitrekenen
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        float distance = 0;
        for (size_t i = 0; i < inliers->indices.size (); ++i)
        {

            pcl::PointXYZRGB point;
            point.x = cloud->points[inliers->indices[i]].x;
            point.y = cloud->points[inliers->indices[i]].y;
            point.z = cloud->points[inliers->indices[i]].z;
            // pack r/g/b into rgb
            uint8_t r = 255, g = k*255, b = 0;    // Example: Red color
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            point.rgb = *reinterpret_cast<float*>(&rgb);
            //rgbcloud->points.push_back(point);

            float z_plane = -point.x * coefficients->values[0] - point.y * coefficients->values[1] - coefficients->values[3];
            //cout<<"for i: "<<i<<" --> zplane: "<<z_plane<<" --> z: "<<point.z<<" grootte--> "<< pow(point.z - z_plane, 2)<<endl;

            rgbcloud->points.push_back(point);
            distance += pow(point.z - z_plane,2);
        }
        distance /= inliers->indices.size ();
        distance = sqrt(distance);

        cout<<"RMS error from plane: "<<distance*1000<<" mm"<<endl;
        cout<<"Relative error: "<< distance/diagonaal * 100<<"%"<<endl;
        cloud_collection.push_back(rgbcloud);

        viewer->addPlane(*coefficients, "plane" + conv.str());

        viewer = rgbVis(cloud_collection[k], viewer, conv.str());
      //--------------------
      // -----Main loop-----
      //--------------------

    }


    //viewer = rgbVis(cloud_collection[1], viewer, "1");
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    return 0;
}


int rms_error_top_plane()
{
     vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_collection; //(new pcl::PointCloud<pcl::PointXYZ>);
   // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_en(new pcl::PointCloud<pcl::PointXYZ>);

    vector<string> files;

    files.push_back("en.ply");
    files.push_back("sl.ply");

    pcl::PLYReader plyreader;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    for(int k=0; k<files.size() ; k++)
    {

        ostringstream conv;
        conv << k;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        plyreader.read(files[k], *cloud);
        cout<<files[k]<<endl;
        cloud->width  = cloud->points.size();
        cloud->height = 1;
        cloud->points.resize (cloud->width * cloud->height);

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);

        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            return (-1);
        }

        while(abs(coefficients->values[1] ) > 0.01)
        {
            cout<<"rotate x"<<endl;
            ///Build our own rotation matrix
            Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
            float theta;
            // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
            theta = asin(coefficients->values[1]);
            //rotate around X-Axis
            transform_1 (1,1) = cos (theta);
            transform_1 (1,2) = -sin(theta);
            transform_1 (2,1) = sin (theta);
            transform_1 (2,2) = cos (theta);
            //    (row, column)

            pcl::transformPointCloud (*cloud, *cloud, transform_1);

            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);

            if (inliers->indices.size () == 0)
            {
                PCL_ERROR ("Could not estimate a planar model for the given dataset.");
                return (-1);
            }

            std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                              << coefficients->values[1] << " "
                                              << coefficients->values[2] << " "
                                              << coefficients->values[3] << std::endl;

        }

        while(abs(1.0 - coefficients->values[2] ) > 0.01)
        {
            ///Build our own rotation matrix
            Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
            float theta;
            // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
            theta = (M_PI_2) - asin(coefficients->values[2]);
            cout<<"asin: "<<theta<<endl;
            //rotate around Y-Axis
            transform_1 (0,0) = cos (theta);
            transform_1 (2,0) = -sin(theta);
            transform_1 (0,2) = sin (theta);
            transform_1 (2,2) = cos (theta);
            //    (row, column)

            pcl::transformPointCloud (*cloud, *cloud, transform_1);

            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);

            if (inliers->indices.size () == 0)
            {
                PCL_ERROR ("Could not estimate a planar model for the given dataset.");
                return (-1);
            }

            std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                              << coefficients->values[1] << " "
                                              << coefficients->values[2] << " "
                                              << coefficients->values[3] << std::endl;

        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

        for(int i =0 ; i< cloud->points.size(); i++)
        {
            if(cloud->points[i].z < abs(coefficients->values[3]+0.030))
            {
                cloud2->points.push_back(cloud->points[i]);
            }
        }

        ///Find plane of top of box in filtered cloud2
        cloud2->width  = cloud2->points.size();
        cloud2->height = 1;
        cloud2->points.resize (cloud2->width * cloud2->height);

        pcl::ModelCoefficients::Ptr coefficients2 (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg2;
        // Optional

        seg2.setOptimizeCoefficients (true);

        // Mandatory
        seg2.setModelType(pcl::SACMODEL_PLANE);
        seg2.setMethodType(pcl::SAC_RANSAC);
        seg2.setDistanceThreshold (0.01);

        seg2.setInputCloud (cloud2);
        seg2.segment (*inliers2, *coefficients2);

        if (inliers2->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            return (-1);
        }

        while(coefficients2->values[2] < 0.9)       ///As long as this is true, the plane we found isn't perpendicular to the Z-Axis, so we remove the points from the cloud
        {
            for (int i = inliers2->indices.size()-1; i >= 0; i--)
            {
                cloud2->points.erase(cloud2->points.begin() + inliers2->indices[i]);
            }

            seg2.setInputCloud (cloud2);
            seg2.segment (*inliers2, *coefficients2);

            if (inliers2->indices.size () == 0)
            {
                PCL_ERROR ("Could not estimate a planar model for the given dataset.");
                return (-1);
            }
        }

        std::cerr << "Model coefficients: " << coefficients2->values[0] << " "
                                              << coefficients2->values[1] << " "
                                              << coefficients2->values[2] << " "
                                              << coefficients2->values[3] << std::endl;

        ///Grondvlak tekenen + gemiddelde afwijking uitrekenen
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        float distance = 0;
        for (size_t i = 0; i < inliers2->indices.size (); ++i)
        {

            pcl::PointXYZRGB point;
            point.x = cloud2->points[inliers2->indices[i]].x;
            point.y = cloud2->points[inliers2->indices[i]].y;
            point.z = cloud2->points[inliers2->indices[i]].z;
            // pack r/g/b into rgb
            uint8_t r = 255, g = k*255, b = 0;    // Example: Red color
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            point.rgb = *reinterpret_cast<float*>(&rgb);
            //rgbcloud->points.push_back(point);

            float z_plane = -point.x * coefficients2->values[0] - point.y * coefficients2->values[1] - coefficients2->values[3];
            //cout<<"for i: "<<i<<" --> zplane: "<<z_plane<<" --> z: "<<point.z<<" grootte--> "<< pow(point.z - z_plane, 2)<<endl;

            rgbcloud->points.push_back(point);
            distance += pow(point.z - z_plane,2);
        }
        distance /= inliers2->indices.size ();
        distance = sqrt(distance);

        cout<<"RMS error from plane: "<<distance*1000<<" mm"<<endl;
        cout<<"Measured height of the object: "<<(-coefficients->values[3] + coefficients2->values[3])*1000<<"mm"<<endl;
        cloud_collection.push_back(rgbcloud);

        viewer = rgbVis(rgbcloud, viewer, conv.str());
        viewer->addPlane(*coefficients2, "plane"+conv.str());


    }
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    return 0;
}

int main (int argc, char** argv)
{
    int res;
    res = rms_error_ground_plane();
    res = rms_error_top_plane();
    return res;
}
