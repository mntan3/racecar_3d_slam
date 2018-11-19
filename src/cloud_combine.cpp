#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

ros::Publisher pub;
int cloud_size;

class CloudCombine
{
    public:
        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);
        pcl::PointCloud<pcl::PointXYZ> cloud_combined;
};

void CloudCombine::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Containers for data
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*input, cloud);


    // Perform the actual combining
    cloud_combined += cloud;
    
    // Cut part of cloud
    if (cloud_combined.size() >= cloud_size) {
        cloud_combined.erase(cloud_combined.begin(), cloud_combined.end()-cloud_size);
    }

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_combined, output);
    output.header.frame_id="laser";

    pub.publish(output);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_combine_server");
    ros::NodeHandle nh;
    nh.getParam("/keep_points", cloud_size);

    CloudCombine c;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("combine_in_points", 1, &CloudCombine::cloudCallback, &c);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("combined_points", 1);

    ros::spin();
    return 0;
}
