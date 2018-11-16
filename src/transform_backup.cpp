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

class CloudTransform
{
    public:
        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);
};

void CloudTransform::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    geometry_msgs::TransformStamped transform;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    try
    {
        transform = tfBuffer.lookupTransform("cartographer_map", "laser", ros::Time::now(), ros::Duration(3.0));
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("Couldn't transform %s", ex.what());
        return;
    }
    sensor_msgs::PointCloud2 afterTransform;

    tf2::doTransform(*input, afterTransform, transform);
    pub.publish(afterTransform);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_transform_server");
    ros::NodeHandle nh;

    CloudTransform c;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("in_points", 1, &CloudTransform::cloudCallback, &c);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("out_points", 1);

    ros::spin();
    return 0;
}
