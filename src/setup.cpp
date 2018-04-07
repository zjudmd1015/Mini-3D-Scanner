#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int64.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <Eigen/Eigen>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGBA PointT;
// static int pcd_index = 0;
// static char gotDataFlag = 0;// could use a 'class' to reduce this global variable
static pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT>);

pcl::PointCloud<PointT>::Ptr
cloud_filter(pcl::PointCloud<PointT>::Ptr &cloud);

void
callback(sensor_msgs::PointCloud2 cloud_raw)
{
    // cloud_raw is PC data from Kinect V2;
    // static int pcd_index = 0;


    ROS_INFO("Setuping...");

    // change PC format from PointCloud2 to pcl::PointCloud<PointT>
    pcl::fromROSMsg(cloud_raw, *cloud_ptr);

    // crop, segment, filter
    cloud_ptr = cloud_filter(cloud_ptr);

}


int
main (int argc, char **argv)
{
    ros::init (argc, argv, "setup_scanner");

    ros::NodeHandle nh; // can sub and pub use the same NodeHandle?
    ros::Subscriber sub = nh.subscribe("/kinect2/qhd/points", 1 , callback);
    // ros::Publisher pub = nh.advertise<std_msgs::Inst64> ("pcd_save_done", 1);

    // set up visualizer
    pcl::visualization::PCLVisualizer viewer("viewer");
    viewer.setBackgroundColor (0, 0, 0);
    viewer.addCoordinateSystem (0.5);
    viewer.setCameraPosition(0,0,-3.0,0,-1,0);
    viewer.initCameraParameters ();

    viewer.addPointCloud<PointT> (cloud_ptr, "wholeCloud");

    // ros::Rate loop_rate(10);
    while (ros::ok()  && !viewer.wasStopped())
    {
        viewer.spinOnce(50);
        viewer.updatePointCloud( cloud_ptr, "wholeCloud" );

        ros::spinOnce();
        // loop_rate.sleep();
    }
    return 0;
}


pcl::PointCloud<PointT>::Ptr
cloud_filter(pcl::PointCloud<PointT>::Ptr &cloud)
{
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

  //****************************************************//
    // Create the filtering object - passthrough
    pcl::PassThrough<PointT> passz;
    passz.setInputCloud (cloud);
    passz.setFilterFieldName ("z");
    passz.setFilterLimits (0.75, 1.0);


    // passz.setFilterLimits (-2.0, 4.0);
    //pass.setFilterLimitsNegative (true);
    passz.filter (*cloud_filtered);

    pcl::PassThrough<PointT> passy;
    passy.setInputCloud (cloud_filtered);
    passy.setFilterFieldName ("y");
    passy.setFilterLimits (-0.1, 0.22);

    // passy.setFilterLimits (-2.0, 2.0);
    //pass.setFilterLimitsNegative (true);
    passy.filter (*cloud_filtered);

    pcl::PassThrough<PointT> passx;
    passx.setInputCloud (cloud_filtered);
    passx.setFilterFieldName ("x");
    passx.setFilterLimits (-0.18, 0.18);

    // passx.setFilterLimits (-3.0, 3.0);
    //pass.setFilterLimitsNegative (true);
    passx.filter (*cloud_filtered);
  //****************************************************//



  //****************************************************//
    // // segment ground
    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // // Create the segmentation object
    // pcl::SACSegmentation<PointT> seg;
    // // Optional
    // seg.setOptimizeCoefficients (true);
    // // Mandatory
    // seg.setModelType (pcl::SACMODEL_PLANE);  // plane
    // seg.setMethodType (pcl::SAC_RANSAC);
    // seg.setDistanceThreshold (0.010);

    // seg.setInputCloud (cloud_filtered);
    // seg.segment (*inliers, *coefficients);

    // pcl::ExtractIndices<PointT> extract;
    // extract.setInputCloud(cloud_filtered);
    // extract.setIndices(inliers);
    // extract.setNegative(true);
    // extract.filter(*cloud_filtered);
  //****************************************************//


  //****************************************************//
    // Create the filtering object - StatisticalOutlierRemoval filter
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (cloud_filtered);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);

  //****************************************************//

    // pcl::PointCloud<PointT>::Ptr cloud_write (new pcl::PointCloud<PointT>);
    // cloud_write.width = cloud_filtered.points.size();
    // cloud_write.height = 1;
    // cloud_write.is_dense = false;

    return cloud_filtered;

}
