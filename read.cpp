#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s \n", argv[1]);
    return (-1);
  }

  std::cout << "Loaded " << cloud->width << " x " << cloud->height << std::endl;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud);

  pcl::PointXYZ searchPoint(0, 0, 0);
  float radius = 1.5;

  std::vector<int> pointIdxRadiusSearch; //to store index of surrounding points 
  std::vector<float> pointRadiusSquaredDistance; // to store distance to surrounding points

  kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
    cloud_cluster->points.push_back(cloud->points[ pointIdxRadiusSearch[i] ]);
  cloud_cluster->width = cloud_cluster->points.size ();
  cloud_cluster->height = 1;
  cloud_cluster->is_dense = true;

  pcl::io::savePCDFileASCII ("cluster.pcd", *cloud_cluster);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trim (new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < cloud->points.size (); ++i) {
    if (cloud->points[i].y > 0)
      cloud_trim->points.push_back(cloud->points[i]);
  }
  cloud_trim->width = cloud_trim->points.size ();
  cloud_trim->height = 1;
  // cloud_cluster->is_dense = true;

  pcl::io::savePCDFileASCII ("trim.pcd", *cloud_trim);

  pcl::visualization::PCLVisualizer viewer ("PCL visualizer");
  viewer.addPointCloud (cloud_trim, "cloud"); // Method #1
  viewer.addCoordinateSystem (1.0, "axis", 0);
  viewer.setBackgroundColor (0.05, 0.05, 0.05, 0);  // Setting background to a dark grey
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

  while (!viewer.wasStopped ()) {
    viewer.spinOnce ();
  }
  return (0);

}