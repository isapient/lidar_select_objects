#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

class PointCloudFilterNode {
public:
    PointCloudFilterNode() {
        // Initialize ROS node handle
        nh_ = ros::NodeHandle("~"); // Private node handle for parameters

        // Subscribe to the input point cloud topic
        input_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/mbuggy/os1/points", 1,
                                                                 &PointCloudFilterNode::pointCloudCallback, this);

        // Create publishers for separated point clouds
        noise_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/lidar_filter/objects", 1);
        object_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/lidar_filter/non_objects", 1);

        // Set filtering parameters
        // You can add parameters for filtering here

        // Initialize other variables as needed
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud) {
        // Convert ROS point cloud to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input_cloud, *pcl_cloud);
        
        // Create separate point cloud containers for objects and non-objects
        pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr noise_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Loop through each point in the input cloud
        for (const pcl::PointXYZ& point : pcl_cloud->points) {
        // Check the z-coordinate (height) of the point
            if (point.z > 0.5f) {
                // If the z-coordinate is higher than 1 meter, consider it an object
                object_cloud->points.push_back(point);
            } else {
                // Otherwise, consider it a non-object
                noise_cloud->points.push_back(point);
            }
        } 

        // Publish the separated point clouds as ROS messages
        sensor_msgs::PointCloud2 object_ros_cloud;
        sensor_msgs::PointCloud2 noise_ros_cloud;

        pcl::toROSMsg(*object_cloud, object_ros_cloud);
        pcl::toROSMsg(*noise_cloud, noise_ros_cloud);


        // Set the header information for ROS messages
        object_ros_cloud.header = input_cloud->header;
        noise_ros_cloud.header = input_cloud->header;

        // Publish the separated point clouds
        object_cloud_pub_.publish(object_ros_cloud);
        noise_cloud_pub_.publish(noise_ros_cloud);
        }

private:
    ros::NodeHandle nh_;
    ros::Subscriber input_cloud_sub_;
    ros::Publisher noise_cloud_pub_;
    ros::Publisher object_cloud_pub_;

    // Define additional member variables and parameters as needed
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_filter_node");

    PointCloudFilterNode filter_node;

    ros::spin();

    return 0;
}