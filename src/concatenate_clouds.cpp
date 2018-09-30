#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    ROS_INFO("Cloud: width =%d, height=%d\n", msg->width, msg->height);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "concatenator");
    ros::NodeHandle n;

    ros::Subscriber sub_message = n.subscribe("velodyne_points", 1000, pointcloudCallback);
    ros::Publisher point_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("all_points", 1000);

    ros::spin();
    return 0;
}
