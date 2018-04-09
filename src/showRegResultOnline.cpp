#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGBA PointT;

// ***********************
// ******** PCLVisualizer
// ***********************
int
main (int argc, char **argv)
{
    ros::init (argc, argv, "showRegResultOnline");
    ros::NodeHandle nh;

    std::string filename = "/home/dylan2/catkin_ws/src/scanner/data/result/registerResult.pcd";
    pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT>);

    // set up visualizer
    pcl::visualization::PCLVisualizer viewer("viewer");
    viewer.setBackgroundColor (0, 0, 0);
    // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "wholeCloud");
    viewer.addCoordinateSystem (0.5);
    viewer.setCameraPosition(0,0,-3.0,0,-1,0);
    // viewer.setCameraPosition(0,-0.5,1.0,0,0,-1);
    // viewer.setFullScreen(true);
    viewer.initCameraParameters ();

    viewer.addPointCloud<PointT> (cloud_ptr, "wholeCloud");

    ros::Rate loop_rate(2);
    loop_rate.sleep();              // sleep two second at the beginning
    ros::Rate loop_rate_runtime(2);

    while (ros::ok() && !viewer.wasStopped())
    {
        // pcl::io::loadPCDFile<PointT> (filename, *cloud_ptr);
        if (pcl::io::loadPCDFile<PointT> (filename, *cloud_ptr) == -1){
          // PCL_ERROR ("Couldn't read PCD file \n");
          continue;
        }
       // std::cout << (cloud_ptr->size()) << endl;
        if (cloud_ptr->size() < 66) continue;

        viewer.spinOnce(777);
        // viewer.spin();
        viewer.updatePointCloud( cloud_ptr, "wholeCloud" );
        // viewer.setCameraPosition(0,-0.5,1.0,0,0,-1);
        // loop_rate_runtime.sleep();
    }

    return 0;
}

// ********************
// ******** Cloudviewer
// ********************

// int
// main (int argc, char **argv)
// {
//     ros::init (argc, argv, "showRegResultOnline");
//     ros::NodeHandle nh;

//     std::string filename = "/home/dylan2/catkin_ws/src/temp/pointCloudInRviz/data/result/registerResult.pcd";
//     pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT>);
//     pcl::visualization::CloudViewer viewer("viewer");

//     ros::Rate loop_rate(0.5);
//     loop_rate.sleep();              // sleep two second at the beginning
//     ros::Rate loop_rate_runtime(2);

//     while (ros::ok())
//     {
//         // pcl::io::loadPCDFile<PointT> (filename, *cloud_ptr);
//         if (pcl::io::loadPCDFile<PointT> (filename, *cloud_ptr) == -1){
//           // PCL_ERROR ("Couldn't read PCD file \n");
//           continue;
//         }

//         viewer.showCloud( cloud_ptr );
//         loop_rate_runtime.sleep();
//     }

//     return 0;
// }




