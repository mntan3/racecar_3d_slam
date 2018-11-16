#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"

class PoseDrawer
{
public:
  PoseDrawer() :
    tf2_(buffer_),  target_frame_("cartographer_map"),
    tf2_filter_(point_sub_, buffer_, target_frame_, 10, 0)
  {
    point_sub_.subscribe(n_, "velodyne_points", 10);
    pub = n_.advertise<sensor_msgs::PointCloud2> ("out_points", 1);
    tf2_filter_.registerCallback( boost::bind(&PoseDrawer::msgCallback, this, _1) );
  }

  //  Callback to register with tf2_ros::MessageFilter to be called when transforms are available
  void msgCallback(const sensor_msgs::PointCloud2ConstPtr& point_ptr) 
  {
      geometry_msgs::TransformStamped transform;
    try 
    {
       transform = buffer_.lookupTransform("cartographer_map", "laser", ros::Time::now(), ros::Duration(3.0));
      
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
    }
    sensor_msgs::PointCloud2 afterTransform;
    tf2::doTransform(*point_ptr, afterTransform, transform);

    pub.publish(afterTransform);

  }

private:
  std::string target_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;
  ros::NodeHandle n_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> point_sub_;
  ros::Publisher pub;
  tf2_ros::MessageFilter<sensor_msgs::PointCloud2> tf2_filter_;

};


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "pose_drawer"); //Init ROS
  PoseDrawer pd; //Construct class
  ros::spin(); // Run until interupted 
  return 0;
};

