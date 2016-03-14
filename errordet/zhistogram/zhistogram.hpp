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
using namespace pcl;

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, string cloudname);
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, string cloudname);
bool rotateCloud(PointCloud<PointXYZ>::Ptr cloud);
void tonen(Mat image, String naam);
bool sort_z(vector<int>& number_of_z_values, PointCloud<PointXYZ>::Ptr cloud);
void visualize_z(vector<int> number_of_z_values );
void print(vector<int> number_of_z_values, string path);

