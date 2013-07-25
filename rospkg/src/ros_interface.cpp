#include "door_finder/edge_detector.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <door_finder/SimpleConfig.h>
#include <ros/ros.h>



static EdgeDetector * detector;
static ros::Publisher pub;

void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    sensor_msgs::PointCloud2 output;
    pcl::PointCloud< pcl::PointXYZRGBA >::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA> );
    pcl::fromROSMsg (*input, *cloud);
    detector->inputPointCloud( cloud, false );
    //pub.publish (output);
}   


int main (int argc, char** argv)
{
    //initialize the edge detector.
    detector = new EdgeDetector( "../config.h" );

    // Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");
    
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    // Spin
    ros::spin ();
}
