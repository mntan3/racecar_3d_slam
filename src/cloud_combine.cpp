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
        pcl::PCLPointCloud2 cloud_combined;
};

void CloudCombine::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Containers for data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;

    // Convert to PCL data type
    pcl_conversions::toPCL(*input, *cloud);

    // Perform the actual combining
    pcl::concatenatePointCloud(cloud_combined, *cloud, cloud_combined);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_combined, output);
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
