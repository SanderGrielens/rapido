#include "zhistogram.hpp"

void tonen(Mat image, String naam)              // afbeelding tonen op scherm a.d.h.v. afbeelding en naam venster
{
    namedWindow( naam, WINDOW_NORMAL );
    resizeWindow(naam, 1200,800);
    imshow( naam, image );
    waitKey(0);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, string cloudname)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, cloudname);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloudname);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,string cloudname)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, cloudname);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudname);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

bool rotateCloud(PointCloud<PointXYZ>::Ptr cloud)
{
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
            return false;
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
            return false;
        }
    }



    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                            << coefficients->values[1] << " "
                                            << coefficients->values[2] << " "
                                            << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    return true;
}

bool sort_z(vector<int>& number_of_z_values, PointCloud<PointXYZ>::Ptr cloud)
{
    int maxvalue = number_of_z_values.size();
    for(int i=0; i<cloud->points.size(); i++)
    {
        int z = cloud->points[i].z*1000;
        if(z > number_of_z_values.size())
        {
            return false;
        }
        int holder = number_of_z_values[z];
        ++holder;
        number_of_z_values[z] = holder;
        //cout<<number_of_z_values[z]<<endl;
    }
    return true;
}

void visualize_z(vector<int>number_of_z_values)
{
    int rows = 0;
    for(int i=0; i<number_of_z_values.size(); i++)
    {
        rows = (rows < number_of_z_values[i] ? number_of_z_values[i] : rows);
    }

    int cols =  number_of_z_values.size();

    cout<<"BLAAAAAAAAAAAAAAAAAAAAAAAA "<< rows<<" "<<cols<<endl;
    Mat vis = Mat::zeros(rows, cols,CV_8UC1);
    tonen(vis, "z_waardes");
}

void print(vector<int> number_of_z_values, string path)
{
    ofstream zhistogram (path.c_str());         //Opening file to print info to
    zhistogram << "z;aantal" << endl;          //Headings for file
    int kleinste = number_of_z_values.size()+100;
    int grootste = 0;
    for(int i =0; i<number_of_z_values.size(); i++)
    {
        if(number_of_z_values[i] > 0 && kleinste > i)
            kleinste = i;
        if(number_of_z_values[i] > 0 && grootste < i)
            grootste = i;
    }

    cout<<"kleinste "<<kleinste<<" grootste "<<grootste<<endl;


    for (int i =kleinste; i<=grootste; i++)
    {
      //if(number_of_z_values[i]> 0)
        zhistogram << grootste - (i) << ";" << number_of_z_values[i]<<endl;                                 //Printing to file
    }
}

