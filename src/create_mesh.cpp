#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include "shape_msgs/Mesh.h"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Timer.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Scale_space_surface_reconstruction_3<Kernel> Reconstruction;
typedef CGAL::Scale_space_reconstruction_3::Weighted_PCA_smoother<Kernel> Smoother;
typedef CGAL::Scale_space_reconstruction_3::Alpha_shape_mesher<Kernel> Mesher;
typedef Reconstruction::Facet_const_iterator Facet_iterator;
typedef Reconstruction::Point_const_iterator Point_iterator;
typedef Kernel::Point_3 Point;
typedef CGAL::Timer Timer;

ros::Publisher pub;

class MeshCreator
{
    public:
        void pointsCallback(const sensor_msgs::PointCloud2ConstPtr& input);
};

void MeshCreator::pointsCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Containers for data
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*input, cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p = cloud.makeShared();

    // Convert to CGAL
    std::vector<Point> points;
    for(pcl::PointCloud<pcl::PointXYZ>::iterator it=cloud_p->begin(); it != cloud_p->end(); it++) 
    {
        points.push_back(Point(it->x, it->y, it->z));
    }
    if (points.size() == 0) {
        return;
    }

    // Construct mesh
    Reconstruction reconstruct (points.begin(), points.end());
    
    Smoother smoother(12, 300);
    reconstruct.increase_scale(4, smoother);
    Mesher mesher(smoother.squared_radius(), false, false);

    reconstruct.reconstruct_surface(mesher);

    // Convert to ROS
    std::vector<geometry_msgs::Point> mesh_points;
    std::vector<shape_msgs::MeshTriangle> mesh_triangles;
    
    for (Facet_iterator it = reconstruct.facets_begin(); it != reconstruct.facets_end(); ++it) {
        shape_msgs::MeshTriangle triangle_msg;
        triangle_msg.vertex_indices = *it;
        mesh_triangles.push_back(triangle_msg);
    }

    for (Point_iterator it=reconstruct.points_begin(); it != reconstruct.points_end(); ++it) {
        geometry_msgs::Point point_msg;
        CGAL::Point_3<CGAL::Epick> a = *it;
        point_msg.x = a.x();
        point_msg.y = a.y();
        point_msg.z = a.z();
        mesh_points.push_back(point_msg);
    }

    shape_msgs::Mesh msg;
    msg.triangles = mesh_triangles;
    msg.vertices = mesh_points;
    pub.publish(msg);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mesh_creator");
    ros::NodeHandle nh;

    MeshCreator m;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("downsampled_points", 1, &MeshCreator::pointsCallback, &m);
    pub = nh.advertise<shape_msgs::Mesh> ("mesh", 1);

    ros::spin();
    return 0;
}
