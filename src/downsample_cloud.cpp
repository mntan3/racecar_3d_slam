#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"

ros::Publisher pub;
float leaf_size;

class DownsampleCloud
{
    public:
        void pointsCallback(const sensor_msgs::PointCloud2ConstPtr& input);
};

void DownsampleCloud::pointsCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Containers for data
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;

    pcl::fromROSMsg(*input, cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p = cloud.makeShared();

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_p);
    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    sor.filter (cloud_filtered);
    
    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_filtered, output);
    ROS_INFO("width: %d", output.width);
    output.header.frame_id = "laser";
    pub.publish(output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "downsample_node");
    ros::NodeHandle nh;
    nh.getParam("/leaf_size", leaf_size);

    DownsampleCloud c;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("combined_points", 1000, &DownsampleCloud::pointsCallback, &c);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("downsampled_points", 1);

    ros::spin();
    return 0;
}
