#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>

using namespace cv;

std::vector<Vec3f> lidar_points, image_points;

void pp_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
  std::cout << "Picking event active" << std::endl;
    if (event.getPointIndex()!= -1)
    {
        float x,y,z;
        event.getPoint (x,y,z);
        std::cout << x<< ", " << y<< ", " << z << std::endl;
    }
}

double distance(double x1, double y1, double x2, double y2) {
    return sqrt ((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

int main (int argc, char** argv)
{
    double ratio;

    lidar_points.push_back(Vec3f(0.136657, 0.00288897, 0.0808431));
    lidar_points.push_back(Vec3f(-0.151574, -0.00122424, 0.124404));
    image_points.push_back(Vec3f(731, 417, 85.1469));
    image_points.push_back(Vec3f(419, 423, 122.674));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr image_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    float fx = 3374.12, fy = 3384.41, Ox = 2049.61, Oy = 1457.54;

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file %s \n", argv[1]);
        return (-1);
    }

    Mat image, image_gray;
    image = imread (argv[2], CV_LOAD_IMAGE_COLOR);
    if (!image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl;
        return -1;
    }

    /*
    for (int i = 0; i < 4032; i+=10) {
        for (int j = 0; j < 3024; j+=10) {
            pcl::PointXYZRGB p;
            p.x = round (i * 0.8);
            p.z = round (j * 0.8);
            p.y = 2;
            Vec3b intensity = image.at<Vec3b>(j, i);
            p.r = intensity.val[2];
            p.g = intensity.val[1];
            p.b = intensity.val[0];
            image_cloud->points.push_back(p);
        }
    }
    */
    
    
    lidar_points[0][0] = lidar_points[0][0] * fx + 0.804994 * Ox;
    lidar_points[0][1] = lidar_points[0][1] * fy + 0.804994 * Oy;
    lidar_points[1][0] = lidar_points[1][0] * fx + 0.808424 * Ox;
    lidar_points[1][1] = lidar_points[1][1] * fy + 0.808424 * Oy;
    ratio = distance(image_points[0][0], image_points[0][1], image_points[1][0], image_points[1][1]) /
        distance(lidar_points[0][0], lidar_points[0][1], lidar_points[1][0], lidar_points[1][1]);
    
    /*    
    double Z = 0.804994;
    double tz = lidar_points[0][2] * fx / (image_points[0][2] - Z);
    double tx = (image_points[0][0] - Ox) * (Z + tz) / fx - lidar_points[0][0];
    double ty = (image_points[0][1] - Oy) * (Z + tz) / fx - lidar_points[0][1];
    std::cout << tx << "," << tz << "," << ty << std::endl;
    */
    // Projection

    /*
    for (int i = 0; i < cloud->points.size(); i++) {
        double x = (cloud->points[i].x * fx + cloud->points[i].y * Ox) * ratio;
        double y = (0 - (cloud->points[i].z * fy + cloud->points[i].y * Oy)) * ratio;
        x = round (x + image_points[0][0] - lidar_points[0][0] * ratio);
        y = round (y + image_points[0][1] + lidar_points[0][1] * ratio);

        pcl::PointXYZRGB p;
        p.x = cloud->points[i].x;
        // p.x = x / ratio;
        p.y = cloud->points[i].y;
        p.z = cloud->points[i].z;
        // p.z = (y) / ratio;

        if (x < 0 || x > image.cols || y < 0 || y > image.rows)
            continue;
        Vec3b intensity = image.at<Vec3b>(y, x);
        p.r = intensity.val[2];
        p.g = intensity.val[1];
        p.b = intensity.val[0];
        color_cloud1->points.push_back(p);
    }
    std::cout << "Colored cloud 1: " << color_cloud1->size() << std::endl;
    */

    
    // ratio = image_points[1][2] / lidar_points[1][2];
    // Projection
    for (int i = 0; i < cloud->points.size(); i++) {
        double x = (cloud->points[i].x * fx + cloud->points[i].y * Ox) * ratio;
        double y = (0 - (cloud->points[i].z * fy + cloud->points[i].y * Oy)) * ratio;
        x = round (x + image_points[1][0] - lidar_points[1][0] * ratio);
        y = round (y + image_points[1][1] + lidar_points[1][1] * ratio);

        pcl::PointXYZRGB p;
        p.x = cloud->points[i].x;
        // p.x = x / ratio;
        // p.z = (0 - y) / ratio;
        p.y = cloud->points[i].y;
        p.z = cloud->points[i].z;

        if (x < 0 || x > image.cols || y < 0 || y > image.rows)
            continue;
        Vec3b intensity = image.at<Vec3b>(y, x);
        p.r = intensity.val[2];
        p.g = intensity.val[1];
        p.b = intensity.val[0];
        color_cloud2->points.push_back(p);
    }
    std::cout << "Colored cloud 2: " << color_cloud2->size() << std::endl;

    /*
    color_cloud->width = color_cloud->points.size();
    color_cloud->height = 1; 
    color_cloud->is_dense = true;
    pcl::io::savePCDFileASCII ("color.pcd", *color_cloud);
    */

    pcl::visualization::PCLVisualizer viewer ("PCL visualizer");
    // viewer.addCoordinateSystem (1.0, "axis", 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (cloud, 255, 255, 255);
    viewer.setBackgroundColor (0.05, 0.05, 0.05, 0);  // Setting background to a dark grey
    viewer.addPointCloud<pcl::PointXYZ>(cloud, red, "cloud");
    // viewer.addPointCloud<pcl::PointXYZRGB>(color_cloud1, "color cloud 1");
    viewer.addPointCloud<pcl::PointXYZRGB>(color_cloud2, "color cloud 2");
    // viewer.addPointCloud<pcl::PointXYZRGB>(image_cloud, "cloud2");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "color cloud 2");
    // viewer.registerPointPickingCallback (pp_callback,(void*)&viewer); 

    while (!viewer.wasStopped ()) {
        viewer.spinOnce ();
    }
    return (0);
}