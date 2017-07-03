#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/boundary.h>
#include <pcl/PCLPointCloud2.h>

void pp_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
  std::cout << "Picking event active" << std::endl;
  if (event.getPointIndex()!= -1)
  {
   float x,y,z;
   event.getPoint (x,y,z);
   std::cout << x<< ";" << y<<";" << z << std::endl;
 }
}

int main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s \n", argv[1]);
    return (-1);
  }

  	// Narrow the segmentation region
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud);
  pcl::PointXYZ searchPoint (0.018914, 0.796682, 0.0417641);
  float radius = 0.4;
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
  pcl::PointCloud<pcl::PointXYZ>::Ptr search_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
    search_cloud->points.push_back(cloud->points[ pointIdxRadiusSearch[i] ]);
  search_cloud->width = search_cloud->points.size ();
  search_cloud->height = 1;
  
  	// Perform RANSAC on the segmentation region
  std::vector<int> inliers;
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(
    new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (search_cloud));
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
  ransac.setDistanceThreshold(0.02);
  ransac.computeModel();
  ransac.getInliers(inliers);
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud<pcl::PointXYZ> (*search_cloud, inliers, *plane_cloud);

  	// Estimate boundary of the marker board
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>); 
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne; ne.setInputCloud (plane_cloud); 
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ()); 
  ne.setSearchMethod(tree); 
  ne.setRadiusSearch(0.5); 
  ne.compute(*normals); 

  pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est; 
  est.setInputCloud(plane_cloud);
  est.setInputNormals(normals); 
  est.setRadiusSearch(0.05); 
  est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>)); 

  Eigen::Vector4f u; 
  Eigen::Vector4f v; 
  u = Eigen::Vector4f::Zero (); 
  v = Eigen::Vector4f::Zero (); 

  for (size_t i = 0; i < normals->points.size (); ++i) 
  { 
    est.getCoordinateSystemOnPlane (normals->points[i], u, v); 
    pcl::Vector4fMap n4uv = normals->points[i].getNormalVector4fMap (); 
  } 
  pcl::PointCloud<pcl::Boundary> boundary; 
  est.compute(boundary);

  pcl::PointCloud<pcl::PointXYZ>::Ptr boundaryCloud(new pcl::PointCloud<pcl::PointXYZ>); 
  for (int i = 0; i < plane_cloud->points.size(); i++) 
  { 
    if (boundary[i].boundary_point == 1) 
      boundaryCloud->points.push_back(plane_cloud->points[i]); 
  } 
  boundaryCloud->width = boundaryCloud->points.size(); 
  boundaryCloud->height = 1; 
  boundaryCloud->is_dense = true;
  pcl::io::savePCDFileASCII ("edge.pcd", *boundaryCloud);
  // pcl::io::savePCDFileASCII ("edge.pcd", *boundaryCloud);

  // Visualization
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue (plane_cloud, 0, 0, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (boundaryCloud, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white (cloud, 255, 255, 255);

  pcl::visualization::PCLVisualizer viewer ("PCL visualizer");
  viewer.registerPointPickingCallback(pp_callback, (void*)&viewer);
  // viewer.addCoordinateSystem (1.0, "axis", 0);
  viewer.addPointCloud<pcl::PointXYZ>(cloud, white, "cloud");
  viewer.addPointCloud<pcl::PointXYZ>(plane_cloud, blue, "plane cloud");
  viewer.addPointCloud<pcl::PointXYZ>(boundaryCloud, red, "boundary");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "boundary");
  while (!viewer.wasStopped ()) {
    viewer.spinOnce ();
  }
  return (0);
}