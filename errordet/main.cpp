#include <iostream>
#include <math.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
/*#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
*/

using namespace std;


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

int rms_error_ground_plane()
{
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_collection; //(new pcl::PointCloud<pcl::PointXYZ>);
   // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_en(new pcl::PointCloud<pcl::PointXYZ>);

    vector<string> files;

    files.push_back("sl.ply");
    files.push_back("en.ply");

    pcl::PLYReader plyreader;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    for(int k=0; k<files.size() ; k++)
    {
        ostringstream conv;
        conv << k;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        plyreader.read(files[k], *cloud);
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
        seg.setDistanceThreshold (0.002);

        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            return (-1);
        }



        while(abs(1.0 - coefficients->values[2] ) > 0.01)
        {
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
            uint8_t r = 255, g = 0, b = 0;    // Example: Red color
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

      //--------------------
      // -----Main loop-----
      //--------------------

    }

    viewer = rgbVis(cloud_collection[0], viewer, "0");
    viewer = rgbVis(cloud_collection[1], viewer, "1");
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

    files.push_back("sl.ply");
    files.push_back("en.ply");

    pcl::PLYReader plyreader;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    for(int k=0; k<files.size() ; k++)
    {

        ostringstream conv;
        conv << k;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        plyreader.read(files[k], *cloud);
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
        seg.setDistanceThreshold (0.002);

        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            return (-1);
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
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

        for(int i =0 ; i< cloud->points.size(); i++)
        {
            if(cloud->points[i].z < abs(coefficients->values[3]+0.032))
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
        seg2.setDistanceThreshold (0.002);

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
            uint8_t r = 255, g = 0, b = 0;    // Example: Red color
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

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer = rgbVis(rgbcloud, viewer, conv.str());
        viewer->addPlane(*coefficients, "plane"+conv.str());

        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
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
