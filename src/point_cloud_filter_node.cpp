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
        // Loop through each point in the input cloud
        // printf("count = %ld\n", input_cloud->points.size());
        // int x = 0;

        for (const pcl::PointXYZ &point : input_cloud->points)
        {
            // printf("(%f; %f; %f) ", point.x, point.y, point.z);
            // x++;
            // if(x%32 == 0)printf("\n");
            // if(x%512 == 0)printf("\n\n");

            // Check the z-coordinate (height) of the point
            const float dist = 0.2f;   // 1 meter
            const float shift = -1.5f; // 1 meter

            if (point.z > shift + dist || point.z < shift - dist)
            {
                // If the z-coordinate is higher than 1 meter, consider it an object
                inlier_cloud->points.push_back(point);
            }
            else
            {
                // Otherwise, consider it a non-object
                outlier_cloud->points.push_back(point);
            }
        }

        // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        // sor.setInputCloud(input_cloud);
        // sor.setMeanK(6);            // Number of neighbors to analyze for each point
        // sor.setStddevMulThresh(1.5); // Standard deviation threshold multiplier

        // pcl::PointCloud<pcl::PointXYZ> inliers;
        // sor.filter(inliers); // Filter inliers (objects)

        // pcl::PointCloud<pcl::PointXYZ> outliers;
        // sor.setNegative(true); // Set to filter outliers (non-objects)
        // sor.filter(outliers);

        // // Copy the results back to the provided inlier and outlier point clouds
        // pcl::copyPointCloud(inliers, *inlier_cloud);
        // pcl::copyPointCloud(outliers, *outlier_cloud);
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

    const float NOT_LEVEL = FLT_MIN;

    float calcGridLevel(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                         const int ring_len,
                         const int x1, const int x2,
                         const int y1, const int y2,
                         const float underlying_th,
                         const float msq_correction,
                         const int min_points)
    {
        float result;

        float avg_z1 = 0;
        float avg_z2 = 0;
        int count = 0;
        for (int i = x1; i < x2; i++)
        {
            for (int j = y1; j < y2; j++)
            {
                float x = input_cloud->points[i + j * ring_len].x;
                float y = input_cloud->points[i + j * ring_len].y;
                float z = input_cloud->points[i + j * ring_len].z;
                if (z == 0 && x == 0 &&  y == 0) continue;
                if (z > underlying_th && underlying_th != NOT_LEVEL) continue;
                
                // printf("(%.1f; %.1f; %.1f) ", x, y, z);

                avg_z1 += z;
                avg_z2 += z * z;
                count++;
            }
        }
        if (count >= min_points)
        {
            avg_z1 /= count;
            avg_z2 /= count;
            float msq = sqrt(avg_z2 - avg_z1 * avg_z1);
            result = avg_z1 - msq_correction * msq;
        }
        else
        {
            result = NOT_LEVEL;
        }
        return result;
    }

    void applyGridLevel(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &plain_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &other_cloud,
                         const int ring_len,
                         const int x1, const int x2,
                         const int y1, const int y2,
                         const float level) {
        for (int i = x1; i < x2; i++)
        {
            for (int j = y1; j < y2; j++)
            {
                float x = input_cloud->points[i + j * ring_len].x;
                float y = input_cloud->points[i + j * ring_len].y;
                float z = input_cloud->points[i + j * ring_len].z;
                if (z == 0 && x == 0 &&  y == 0) continue;
                if (z > level || level == NOT_LEVEL){
                    other_cloud->points.push_back(input_cloud->points[i + j * ring_len]);
                } else {
                    plain_cloud->points.push_back(input_cloud->points[i + j * ring_len]);
                }
            }
        }
    }


    void findPlainPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &plain_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &other_cloud) {
        const int ring_len = 512;
        const int ring_cnt = 64;
        const int v_num = 32; // in fact for mapping will be used lower half
        const int h_num = 16;
        const int v_block = ring_cnt / v_num; // = 4
        const int h_block = ring_len / h_num; // = 32
        const int min_points = 12;            // about 10%

        float level[h_num][v_num];
        float msq_correction = 0.0;
        for (int y = 0; y < v_num; y++)
        {
            for (int x = 0; x < h_num; x++)
            {
                float under_avg = calcGridLevel(input_cloud,
                                                 ring_len,
                                                 x * h_block, (x + 1) * h_block,
                                                 y * v_block, (y + 1) * v_block,
                                                 NOT_LEVEL,
                                                 msq_correction,
                                                 min_points);
                level[x][y] = under_avg;

                if(under_avg!=NOT_LEVEL){
                    printf("%4.1f ", under_avg);
                } else {
                    printf(" ... ");
                }

                applyGridLevel(input_cloud,
                               plain_cloud,
                               other_cloud,
                               ring_len,
                               x * h_block, (x + 1) * h_block,
                               y * v_block, (y + 1) * v_block,
                               level[x][y]);
            }
            printf("\n");
        }
        printf("\n");
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
        // prefilterOutliers(pcl_cloud, work_cloud, noise_cloud);
        findPlainPoints(pcl_cloud, noise_cloud, work_cloud);

        // Create separate point cloud containers for plain and other points
        pcl::PointCloud<pcl::PointXYZ>::Ptr plain_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr other_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Use the segmentation and extraction method
        // segmentPointCloud(work_cloud, plain_cloud, other_cloud);

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
