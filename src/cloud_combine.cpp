#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

ros::Publisher pub;

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
    int cloud_size = 100000;
    if (cloud_combined.size() >= cloud_size) {
        cloud_combined.erase(cloud_combined.begin(), cloud_combined.end()-cloud_size);
    }

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_combined, output);
    output.header.frame_id = "laser";
    pub.publish(output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_combine_server");
    ros::NodeHandle nh;

    CloudCombine c;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("velodyne_points", 1000, &CloudCombine::cloudCallback, &c);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("combined_points", 1);

    ros::spin();
    return 0;
}
