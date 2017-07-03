#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

int main (int argc, char** argv)
{
	double f = 3400;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1) //* load the file
    	{
      	PCL_ERROR ("Couldn't read file %s \n", argv[1]);
      	return (-1);
    	}

    	for (int i = 0; i < cloud->points.size(); i++) {
    		if (cloud->points[i].y > 0.5)
    			continue;
    		cloud->points[i].x = (f * cloud->points[i].x) / cloud->points[i].y;
    		cloud->points[i].z = (f * cloud->points[i].z) / cloud->points[i].y;
    		cloud->points[i].y = 1;
    	}

    	pcl::visualization::PCLVisualizer viewer ("PCL visualizer");
   	viewer.addCoordinateSystem (1.0, "axis", 0);
    	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (cloud, 255, 255, 255);
    	viewer.setBackgroundColor (0.05, 0.05, 0.05, 0);  // Setting background to a dark grey
    	viewer.addPointCloud<pcl::PointXYZ>(cloud, red, "cloud");

    	while (!viewer.wasStopped ()) {
        viewer.spinOnce ();
    	}
}
