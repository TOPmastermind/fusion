#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/visualization/pcl_visualizer.h>

template<typename PointT>

void remove_inliers(const pcl::PointCloud<PointT> &cloud_in, std::vector<int> inliers_indices,
                        pcl::PointCloud<PointT> &cloud_out)
{
	std::vector<int> outliers_indicies;
     	for (size_t i = 0; i < cloud_in.size(); i++)
      {
      	if (find(inliers_indices.begin(), inliers_indices.end(), i) == inliers_indices.end())
        	{
          		outliers_indicies.push_back(i);
        	}
      }
      pcl::copyPointCloud< pcl::PointXYZ >(cloud_in, outliers_indicies, cloud_out);
}

int main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  	if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1) //* load the file
  	{
    		PCL_ERROR ("Couldn't read file %s \n", argv[1]);
    	return (-1);
  	}

  	// Remove marker board border lines
  	for (int i = 0; i < 4; i++)
	{
	    	pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l(
	      	new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));
	    	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_l(model_l);
	    	ransac_l.setDistanceThreshold(0.03);
	    	ransac_l.computeModel();
	    	std::vector<int> line_inliers;
	    	ransac_l.getInliers(line_inliers);
	    	if (line_inliers.empty())
	    	{
	      	continue;
	    	}
  		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_line (new pcl::PointCloud<pcl::PointXYZ>);
  		remove_inliers(*cloud, line_inliers, *cloud_no_line);
   		*cloud = *cloud_no_line;
	}

  	pcl::PointCloud<pcl::PointXYZ>::Ptr circle_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr circle (new pcl::PointCloud<pcl::PointXYZ>);

  	std::vector<int> inliers_indicies;
	pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>::Ptr model_s(
		new pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>(cloud));
	Eigen::VectorXf coeficients;
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_sphere(model_s);
	ransac_sphere.setDistanceThreshold(0.01);

	// Detect 8cm radius circle
	model_s->setRadiusLimits(0.07, 0.09);
	ransac_sphere.computeModel();
	inliers_indicies.clear();
	ransac_sphere.getInliers(inliers_indicies);
	if (inliers_indicies.size() == 0) {
		std::cout << "Can not detect 8cm circle" << std::endl;
		return 0;
	}
	ransac_sphere.getModelCoefficients(coeficients);
	pcl::copyPointCloud<pcl::PointXYZ> (*cloud, inliers_indicies, *circle);
	pcl::PointXYZ center1 (coeficients(0), coeficients(1), coeficients(2));
	std::cout << "Detect circle center at (" << coeficients(0) << ", " 
	<< coeficients(1) << ", " << coeficients(2) 
	<< "), radius = " << coeficients(3) << std::endl;
	*circle_cloud += *circle;
	circle_cloud->push_back(center1);

	// Detect 12cm radius circle
	model_s->setRadiusLimits(0.11, 0.13);
	ransac_sphere.computeModel();
	inliers_indicies.clear();
	ransac_sphere.getInliers(inliers_indicies);
	if (inliers_indicies.size() == 0) {
		std::cout << "Can not detect 12cm circle" << std::endl; 
		return 0;
	}
	ransac_sphere.getModelCoefficients(coeficients);
	pcl::copyPointCloud<pcl::PointXYZ> (*cloud, inliers_indicies, *circle);
	pcl::PointXYZ center2 (coeficients(0), coeficients(1), coeficients(2));
	std::cout << "Detect circle center at (" << coeficients(0) << ", " 
	<< coeficients(1) << ", " << coeficients(2) 
	<< "), radius = " << coeficients(3) << std::endl;
	*circle_cloud += *circle;
	circle_cloud->push_back(center2);

	circle_cloud->width = circle_cloud->points.size(); 
  	circle_cloud->height = 1; 
  	circle_cloud->is_dense = true;
  	pcl::io::savePCDFileASCII ("circle.pcd", *circle_cloud);

  	// Visualization
  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue (circle_cloud, 0, 255, 0);
  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white (cloud, 255, 255, 255);
  	pcl::visualization::PCLVisualizer viewer ("PCL visualizer");
  	// viewer.addCoordinateSystem (1.0, "axis", 0);
  	viewer.addPointCloud<pcl::PointXYZ>(cloud, white, "cloud");
  	viewer.addPointCloud<pcl::PointXYZ>(circle_cloud, blue, "circle cloud");
  	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "circle cloud");
  	while (!viewer.wasStopped ()) {
  		viewer.spinOnce ();
  	}
  	return (0);
}