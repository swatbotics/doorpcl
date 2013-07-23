#include "../../edge_detector.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <../../SimpleConfig.h>


static EdgeDetector detector;
static ros::Publisher pub;

void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    sensor_msgs::PointCloud2 output;
    detector.inputPointCloud( input, false );
    //pub.publish (output);
}   


int main (int argc, char** argv)
{
    //initialize the edge detector.
    detector = EdgeDetector( "../config.h" );

    // Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    // Spin
    ros::spin ();
}
