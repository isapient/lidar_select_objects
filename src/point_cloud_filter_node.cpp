#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/filters/extract_indices.h>
#include <pcl_msgs/ModelCoefficients.h>
// #include <pcl_ros/point_indices.h>
#include <pcl_ros/segmentation/sac_segmentation.h>
#include <pcl_ros/filters/extract_indices.h>
#include <pcl_ros/filters/statistical_outlier_removal.h>

class PointCloudFilterNode
{
public:
    PointCloudFilterNode()
    {
        // Initialize ROS node handle
        nh_ = ros::NodeHandle("~"); // Private node handle for parameters

        // Subscribe to the input point cloud topic
        input_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/mbuggy/os1/points", 1,
                                                                   &PointCloudFilterNode::pointCloudCallback, this);

        // Create publishers for separated point clouds
        noise_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/lidar_filter/signal", 1);
        object_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/lidar_filter/noise", 1);

        // Set filtering parameters
        // You can add parameters for filtering here

        // Initialize other variables as needed
    }

    void prefilterOutliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &inlier_cloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &outlier_cloud)
    {
        // // Loop through each point in the input cloud
        // for (const pcl::PointXYZ& point : input_cloud->points) {
        // // Check the z-coordinate (height) of the point
        //     if (point.z > 0.5f) {
        //         // If the z-coordinate is higher than 1 meter, consider it an object
        //         inlier_cloud->points.push_back(point);
        //     } else {
        //         // Otherwise, consider it a non-object
        //         outlier_cloud->points.push_back(point);
        //     }
        // } 

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(input_cloud);
        sor.setMeanK(6);            // Number of neighbors to analyze for each point
        sor.setStddevMulThresh(1.5); // Standard deviation threshold multiplier

        pcl::PointCloud<pcl::PointXYZ> inliers;
        sor.filter(inliers); // Filter inliers (objects)

        pcl::PointCloud<pcl::PointXYZ> outliers;
        sor.setNegative(true); // Set to filter outliers (non-objects)
        sor.filter(outliers);

        // Copy the results back to the provided inlier and outlier point clouds
        pcl::copyPointCloud(inliers, *inlier_cloud);
        pcl::copyPointCloud(outliers, *outlier_cloud);
    }

    void segmentPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &plain_cloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &other_cloud)
    {
        // Segment the ground
        pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        plane->values.resize(4); // Make room for a plane equation (ax+by+cz+d=0)

        pcl::SACSegmentation<pcl::PointXYZ> seg;  // Create the segmentation object
        seg.setAxis(Eigen::Vector3f(0., 0., 1.)); // Set the axis along which we need to search for a model perpendicular to
        seg.setEpsAngle((10. * M_PI) / 180.);     // Set maximum allowed difference between the model normal and the given axis in radians
        seg.setOptimizeCoefficients(true);        // Coefficient refinement is required
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setDistanceThreshold(0.05f);
        seg.setInputCloud(input_cloud);
        seg.segment(*inliers, *plane);

        // Extract inliers
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(input_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);   // Extract the inliers
        extract.filter(*plain_cloud); // plain_cloud contains the plane
        extract.setNegative(true);    // Extract the outliers
        extract.filter(*other_cloud); // other_cloud contains the non-plane points
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &input_cloud)
    {
        // Convert ROS point cloud to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input_cloud, *pcl_cloud);

        // Create separate point cloud containers for objects and non-objects
        pcl::PointCloud<pcl::PointXYZ>::Ptr work_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr noise_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Prefilter outliers in the input cloud
        prefilterOutliers(pcl_cloud, work_cloud, noise_cloud);

        // Create separate point cloud containers for plain and other points
        pcl::PointCloud<pcl::PointXYZ>::Ptr plain_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr other_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Use the segmentation and extraction method
        segmentPointCloud(work_cloud, plain_cloud, other_cloud);

        // Publish the separated point clouds as ROS messages
        sensor_msgs::PointCloud2 object_ros_cloud;
        sensor_msgs::PointCloud2 noise_ros_cloud;

        pcl::toROSMsg(*work_cloud, object_ros_cloud);
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_filter_node");

    PointCloudFilterNode filter_node;

    ros::spin();

    return 0;
}
