#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "shape_msgs/Mesh.h"

ros::Publisher pub;

class VisualizeMesh
{
    public:
        void meshCallback(const shape_msgs::MeshConstPtr& inputMesh);
};

void VisualizeMesh::meshCallback(const shape_msgs::MeshConstPtr& inputMesh)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "cartographer_map";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
/*
    marker.pose.position.x = 1;
    marker.pose.position.y = 1;
    marker.pose.position.z = 1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;*/
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    std::vector<geometry_msgs::Point> trianglePoints;
    std::vector<geometry_msgs::Point> vertices = inputMesh->vertices;
    std::vector<shape_msgs::MeshTriangle> triangles = inputMesh->triangles;
    for (std::vector<shape_msgs::MeshTriangle>::iterator it = triangles.begin(); it < triangles.end(); it++)
    {
        trianglePoints.push_back(vertices.at(it->vertex_indices[0]));
        trianglePoints.push_back(vertices.at(it->vertex_indices[1]));
        trianglePoints.push_back(vertices.at(it->vertex_indices[2]));
    }
    marker.points = trianglePoints;
    pub.publish(marker);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mesh_visualize");
    ros::NodeHandle nh;

    VisualizeMesh v;
    ros::Subscriber sub = nh.subscribe<shape_msgs::Mesh> ("mesh", 1000, &VisualizeMesh::meshCallback, &v);
    pub = nh.advertise<visualization_msgs::Marker> ("visualization_marker", 0);

    ros::spin();
    return 0;
}
