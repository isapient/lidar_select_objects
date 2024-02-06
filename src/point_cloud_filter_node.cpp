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
        _nh = ros::NodeHandle("~"); // Private node handle for parameters

        // Subscribe to the input point cloud topic
        _input_cloud_sub = _nh.subscribe<sensor_msgs::PointCloud2>("/mbuggy/os1/points", 1,
                                                                   &PointCloudFilterNode::pointCloudCallback, this);

        // Create publishers for separated point clouds
        _noise_cloud_pub = _nh.advertise<sensor_msgs::PointCloud2>("/lidar_filter/signal", 1);
        _object_cloud_pub = _nh.advertise<sensor_msgs::PointCloud2>("/lidar_filter/noise", 1);

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
                         const int x1, const int x2,
                         const int y1, const int y2,
                         const float underlying_th,
                         const float msq_mult_correction,
                         const float min_correction,
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
            float correction = msq_mult_correction * msq;

            if(min_correction > 0) {
                if(correction < min_correction) correction = min_correction;
            } else if(min_correction < 0) {
                if(correction > min_correction) correction = min_correction; // negative
            }
             
            if(correction <= 0 and correction >= -min_correction) correction = -min_correction;
            result = avg_z1 + correction;
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

        void printGrids(const float level1[], const float level2[], const float level3[])
        {
            for (int y = 0; y < v_num; y++)
            {
                for (int x = 0; x < h_num; x++) {
                    if (level1[x+y*h_num] != NOT_LEVEL) printf("%3d ", (int)(10 * level1[x+y*h_num] + 0.5));
                    else printf(" .. ");
                }
                printf("| ");
                for (int x = 0; x < h_num; x++) {
                    if (level2[x+y*h_num] != NOT_LEVEL) printf("%3d ", (int)(10 * level2[x+y*h_num] + 0.5));
                    else printf(" .. ");
                }
                printf("| ");
                for (int x = 0; x < h_num; x++) {
                    if (level3[x+y*h_num] != NOT_LEVEL) printf("%3d ", (int)(10 * level3[x+y*h_num] + 0.5));
                    else printf(" .. ");
                }
                printf("\n");
            }
            printf("\n");
        }

    void findPlainPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &plain_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &other_cloud) {

        float low_level[h_num*v_num];
        float avg_level[h_num*v_num];
        float th_level[h_num*v_num];
        
        float distillate_sigma_shift = -0.7; // keeping half of samples under average
        float expand_sigma_shift = +7.0;     // capturing back average ground level + over average samples
        float normal_th_sigma_shift = +4.0;   // measuring 3 sigma above average ground level
        float min_correction = 0.05;         // 5 centimeters

        for (int y = 0; y < v_num; y++)
        {
            for (int x = 0; x < h_num; x++)
            {
                low_level[x+y*h_num] = calcGridLevel(input_cloud,
                                                 x * h_block, (x + 1) * h_block,
                                                 y * v_block, (y + 1) * v_block,
                                                 NOT_LEVEL,
                                                 distillate_sigma_shift,
                                                 0,
                                                 min_points);
            }
        }

        for (int y = 0; y < v_num; y++)
        {
            for (int x = 0; x < h_num; x++)
            {
                avg_level[x+y*h_num] = calcGridLevel(input_cloud,
                                                 x * h_block, (x + 1) * h_block,
                                                 y * v_block, (y + 1) * v_block,
                                                 low_level[x+y*h_num],
                                                 expand_sigma_shift,
                                                 min_correction,
                                                 min_points);
            }
        }

        for (int y = 0; y < v_num; y++)
        {
            for (int x = 0; x < h_num; x++)
            {
                th_level[x+y*h_num] = calcGridLevel(input_cloud,
                                                 x * h_block, (x + 1) * h_block,
                                                 y * v_block, (y + 1) * v_block,
                                                 avg_level[x+y*h_num],
                                                 normal_th_sigma_shift,
                                                 min_correction,
                                                 min_points);
            }
        }


        printGrids(low_level, avg_level, th_level);

        for (int y = 0; y < v_num; y++)
        {
            for (int x = 0; x < h_num; x++)
            {
                applyGridLevel(input_cloud,
                               plain_cloud,
                               other_cloud,
                               x * h_block, (x + 1) * h_block,
                               y * v_block, (y + 1) * v_block,
                               th_level[x+y*h_num]);
            }
        }
    }



    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &input_cloud)
    {
        // Convert ROS point cloud to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input_cloud, *pcl_cloud);

        // Create separate point cloud containers for objects and non-objects
        pcl::PointCloud<pcl::PointXYZ>::Ptr work_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr noise_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Separate plain points from other cloud
        findPlainPoints(pcl_cloud, noise_cloud, work_cloud);

        frame_count++;
        printf("frame_count = %d\n", frame_count);

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
        _object_cloud_pub.publish(object_ros_cloud);
        _noise_cloud_pub.publish(noise_ros_cloud);
    }

private:
    ros::NodeHandle _nh;
    ros::Subscriber _input_cloud_sub;
    ros::Publisher _noise_cloud_pub;
    ros::Publisher _object_cloud_pub;
    int frame_count = 0;
        const int ring_len = 512;
        const int ring_cnt = 64;
        const int v_num = 32; // in fact for mapping will be used lower half
        const int h_num = 16;
        const int v_block = ring_cnt / v_num; // = 4
        const int h_block = ring_len / h_num; // = 32
        const int min_points = 12;            // about 10%



};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_filter_node");

    PointCloudFilterNode filter_node;

    ros::spin();

    return 0;
}
