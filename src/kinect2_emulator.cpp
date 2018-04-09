#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZRGBA PointT;

int
main (int argc, char **argv)
{
    int start_index = std::stoi(argv[1]);
    int end_index = std::stoi(argv[2]);

    ros::init (argc, argv, "kinect2_emulator");

    ros::NodeHandle nh;
    ros::Publisher emu_pub = nh.advertise<sensor_msgs::PointCloud2> ("/kinect2/qhd/points", 1);

    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT>);

    std::string dir = "/home/dylan2/catkin_ws/src/scanner/data/emulator/";
    int cloud_index = start_index;
    std::string filename = dir + std::to_string(cloud_index) + ".pcd";


    ros::Rate loop_rate(2);
    while (ros::ok())
    {
        filename = dir + std::to_string(cloud_index) + ".pcd";

        pcl::io::loadPCDFile<PointT> (filename, *cloud_ptr);
        std::cout << filename << std::endl;

        //Convert the cloud to ROS message
        pcl::toROSMsg(*cloud_ptr, output);
        output.header.frame_id = "odom";

        emu_pub.publish(output);
        std::cout << "have published" << std::endl;
        cloud_index++;
        if (cloud_index > end_index)
            break;
        loop_rate.sleep();

    }

    return 0;
}



