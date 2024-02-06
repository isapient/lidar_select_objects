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

#include <vector>

struct GroundPlane
{
    float a, b, c; // z = a*x + b*y + c
};

class Momentums
{
public:
    Momentums()
    {
        cnt = 0;
        x = y = z = 0;
        xx = yy = zz = 0;
        xy = xz = yz = 0;
    }
    void accumulate(float x, float y, float z)
    {
        cnt++;
        this->x += x;
        this->y += y;
        this->z += z;
        this->xx += x * x;
        this->yy += y * y;
        this->zz += z * z;
        this->xy += x * y;
        this->xz += x * z;
        this->yz += y * z;
    }
    void finalize()
    {
        if (cnt == 0)
            return;
        x /= cnt;
        y /= cnt;
        z /= cnt;
        xx = xx / cnt - x * x;
        yy = yy / cnt - y * y;
        zz = zz / cnt - z * z;
        xy = xy / cnt - x * y;
        xz = xz / cnt - x * z;
        yz = yz / cnt - y * z;
    }
    Momentums &operator+=(const Momentums &a)
    {
        cnt += a.cnt;
        x += a.x;
        y += a.y;
        z += a.z;
        xx += a.xx;
        yy += a.yy;
        zz += a.zz;
        xy += a.xy;
        xz += a.xz;
        yz += a.yz;
        return *this;
    }

    int cnt;
    float x, y, z;    // first order
    float xx, yy, zz; // second order
    float xy, xz, yz; // mixed
};

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

    GroundPlane calcGridLevel(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                              float result_grid[],
                              const GroundPlane &plane,
                              const float underlying_th[],
                              const float msq_mult_correction,
                              const float min_abs_correction,
                              const int min_points,
                              const float max_ground_radius)
    {
        Momentums momentums[h_num * v_num]; // zero initialized
        Momentums stats;
        float max_ground_radius_sq = max_ground_radius * max_ground_radius;

        for (int y = 0; y < v_num; y++)
        {
            for (int x = 0; x < h_num; x++)
            {
                const int x1 = h_block * x;
                const int x2 = h_block * (x + 1);
                const int y1 = v_block * y;
                const int y2 = v_block * (y + 1);
                int grid_idx = x + y * h_num;

                float result;

                float avg_z1 = 0;
                float avg_z2 = 0;
                int count = 0;
                int non_zero_count = 0;

                if (underlying_th[grid_idx] == EMPTY_GRID)
                    result_grid[grid_idx] = EMPTY_GRID;

                for (int i = x1; i < x2; i++)
                {
                    for (int j = y1; j < y2; j++)
                    {
                        int pulse_idx = i + j * ring_len;
                        float x = input_cloud->points[pulse_idx].x;
                        float y = input_cloud->points[pulse_idx].y;
                        float z = input_cloud->points[pulse_idx].z;
                        if (z == 0 && x == 0 && y == 0)
                            continue;

                        non_zero_count++;

                        float z0 = plane.a * x + plane.b * y + plane.c;

                        if (z > z0 + underlying_th[grid_idx] && underlying_th[grid_idx] != UNKNOWN_GROUND)
                            continue;

                        if ( x * x + y * y > max_ground_radius_sq)
                            continue;

                        // printf("(%.1f; %.1f; %.1f) ", x, y, z);
                        momentums[grid_idx].accumulate(x, y, z);

                        // avg_z1 += z;
                        // avg_z2 += z * z;
                        // count++;
                    }
                }

                if (momentums[grid_idx].cnt < min_points)
                {
                    if (non_zero_count != 0)
                        result_grid[grid_idx] = UNKNOWN_GROUND;
                    else
                        result_grid[grid_idx] = EMPTY_GRID;
                }
                else
                {
                    result_grid[grid_idx] = 0; // filler - should be calculated later
                    stats += momentums[grid_idx];
                }
            }
        }

        stats.finalize();
        GroundPlane fine_plane;
        float denominator = (stats.xx * stats.yy - stats.xy * stats.xy);
        if (denominator > 0.00001f || denominator < -0.00001f)
        {
            fine_plane.a = (stats.xz * stats.yy - stats.yz * stats.xy) / denominator;
            fine_plane.b = (stats.yz * stats.xx - stats.xz * stats.xy) / denominator;
            fine_plane.c = stats.z - (fine_plane.a * stats.x + fine_plane.b * stats.y);
        }
        else
        {
            fine_plane.a = 0;
            fine_plane.b = 0;
            fine_plane.c = 0;
        }

        for (int y = 0; y < v_num; y++)
        {
            for (int x = 0; x < h_num; x++)
            {
                int grid_idx = x + y * h_num;
                if (result_grid[grid_idx] != EMPTY_GRID && result_grid[grid_idx] != UNKNOWN_GROUND)
                {
                    Momentums over_plane = momentums[grid_idx];
                    over_plane.z -= fine_plane.a * over_plane.x +
                                    fine_plane.b * over_plane.y +
                                    fine_plane.c * over_plane.cnt;

                    momentums[grid_idx].finalize();
                    over_plane.finalize();

                    float msq = sqrt(momentums[grid_idx].zz); // original z MSQ
                    float correction = msq_mult_correction * msq;

                    if ((min_abs_correction > 0 && correction < min_abs_correction) || // apply at least min_correction
                        (min_abs_correction < 0 && correction > min_abs_correction))
                    {
                        correction = min_abs_correction;
                    }

                    result_grid[grid_idx] = over_plane.z + correction; // aligned to plane z level
                }
            }
        }
        return fine_plane;
    }

    void applyGridLevel(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &plain_cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &other_cloud,
                        const GroundPlane &plane,
                        const float underlying_th[])
    {
        for (int y = 0; y < v_num; y++)
        {
            for (int x = 0; x < h_num; x++)
            {
                const int grid_idx = x + y * h_num;
                const float grid_level = underlying_th[grid_idx];

                if (grid_level == EMPTY_GRID)
                    continue;

                const int x1 = h_block * x;
                const int x2 = h_block * (x + 1);
                const int y1 = v_block * y;
                const int y2 = v_block * (y + 1);
                for (int i = x1; i < x2; i++)
                {
                    for (int j = y1; j < y2; j++)
                    {
                        int pulse_idx = i + j * ring_len;
                        float x = input_cloud->points[pulse_idx].x;
                        float y = input_cloud->points[pulse_idx].y;
                        float z = input_cloud->points[pulse_idx].z;
                        if (z == 0 && x == 0 && y == 0)
                            continue;

                        float z0 = plane.a * x + plane.b * y + plane.c;

                        if (z > z0 + grid_level || grid_level == UNKNOWN_GROUND)
                        {
                            other_cloud->points.push_back(input_cloud->points[i + j * ring_len]);
                        }
                        else
                        {
                            plain_cloud->points.push_back(input_cloud->points[i + j * ring_len]);
                        }
                    }
                }
            }
        }
    }

    void printGrids(const float level1[], const float level2[], const float level3[])
    {
        float v;
        for (int y = 0; y < v_num; y++)
        {
            for (int x = 0; x < h_num; x++)
            {
                v = level1[x + y * h_num];
                if (v == UNKNOWN_GROUND)
                    printf(" xx ");
                else if (v == EMPTY_GRID)
                    printf(" .. ");
                else
                    printf("%3d ", (int)(10 * v + 0.5));
            }
            printf("| ");
            for (int x = 0; x < h_num; x++)
            {
                v = level2[x + y * h_num];
                if (v == UNKNOWN_GROUND)
                    printf(" xx ");
                else if (v == EMPTY_GRID)
                    printf(" .. ");
                else
                    printf("%3d ", (int)(10 * v + 0.5));
            }
            printf("| ");
            for (int x = 0; x < h_num; x++)
            {
                v = level3[x + y * h_num];
                if (v == UNKNOWN_GROUND)
                    printf(" xx ");
                else if (v == EMPTY_GRID)
                    printf(" .. ");
                else
                    printf("%3d ", (int)(10 * v + 0.5));
            }
            printf("\n");
        }
        printf("\n");
    }

    void median_3x3(const float src[], float dst[], const int cnt_th, bool process_known_only)
    {
        for (int y = 0; y < v_num; y++)
        {
            for (int x = 0; x < h_num; x++)
            {
                if (process_known_only == true && (src[x + y * h_num] == UNKNOWN_GROUND || src[x + y * h_num] == EMPTY_GRID))
                {
                    dst[x + y * h_num] = src[x + y * h_num];
                    continue;
                }
                std::vector<float> aperture;
                aperture.reserve(9);
                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        int nx = ((x + dx) + h_num) % h_num; // cycled over lidar rings
                        int ny = y + dy;
                        if (ny < 0 || ny >= v_num)
                            continue;
                        float val = src[nx + ny * h_num];
                        if (val == EMPTY_GRID or val == UNKNOWN_GROUND)
                            continue;
                        aperture.push_back(val);
                    }
                }
                if (aperture.size() < cnt_th)
                {
                    if (src[x + y * h_num] == EMPTY_GRID)
                        dst[x + y * h_num] = EMPTY_GRID;
                    else
                        dst[x + y * h_num] = UNKNOWN_GROUND;
                }
                else
                {
                    std::sort(aperture.begin(), aperture.end());
                    int size = aperture.size();
                    if (size % 2 == 1)
                    {
                        dst[x + y * h_num] = aperture[size / 2];
                    }
                    else
                    {
                        dst[x + y * h_num] = (aperture[size / 2 - 1] + aperture[size / 2]) / 2;
                    }
                }
            }
        }
    }

    void copy_empty_mask(const float src[], float dst[])
    {
        for (int p = 0; p < v_num * h_num; p++)
        {
            if (src[p] == EMPTY_GRID)
            {
                dst[p] = EMPTY_GRID;
            }
        }
    }

    void copy_grid(const float src[], float dst[])
    {
        for (int p = 0; p < v_num * h_num; p++)
        {
            dst[p] = src[p];
        }
    }

    void grid_median_recovery(const float low_level[], float low_level_med[])
    {
        const int primary_min_median_neigbours = 3;
        const int secondary_min_median_neigbours = 3;

        float grid_med_1[h_num * v_num];
        float grid_med_2[h_num * v_num];
        median_3x3(low_level, grid_med_1, primary_min_median_neigbours, true);
        median_3x3(grid_med_1, grid_med_2, primary_min_median_neigbours, true);

        float grid_med_3[h_num * v_num];

        for (int i = 0; i < 16; i++)
        {
            median_3x3(grid_med_2, grid_med_3, secondary_min_median_neigbours, false);
            median_3x3(grid_med_3, grid_med_2, secondary_min_median_neigbours, false);
        }
        copy_empty_mask(low_level, grid_med_2);
        copy_grid(grid_med_2, low_level_med);
    }



    void findPlainPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &plain_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &other_cloud)
    {

        float half_level[h_num * v_num];
        float half_level_med[h_num * v_num];

        float quarter_level[h_num * v_num];
        float quarter_level_med[h_num * v_num];

        float th_level[h_num * v_num];
        float th_level_med[h_num * v_num];

        const int min_points_low = 48; // about 36%
        const int min_points_avg = 16;
        const int min_points_high = 8; // about 15%

        const float lowpart_sigma_shift = 0;       // keeping 50% of samples under threshold
        const float distillate_sigma_shift = +0.7; // keeping 75% of remaining samples (37% of total samples)
        const float normal_th_sigma_shift = +4.5;  // more than 3 sigma above average ground level

        const float no_correction = 0.0;   // 0 centimeters
        const float min_correction = 0.05; // add 5 centimeters
        const float big_correction = 0.20; // add 20 centimeters over ground level

        const float max_ground_radius = 30.0f; // 30 meters

        GroundPlane horizontal_plane; // inited by zero coefficients
        float unknown_levels[h_num * v_num];
        for (int i = 0; i < v_num * h_num; i++)
            unknown_levels[i] = UNKNOWN_GROUND;

        GroundPlane dirty_plane = calcGridLevel(input_cloud,
                                                half_level,
                                                horizontal_plane,
                                                unknown_levels,
                                                lowpart_sigma_shift,
                                                no_correction,
                                                min_points_low,
                                                max_ground_radius);

        // printGrids(low_level, low_level, low_level);

        grid_median_recovery(half_level, half_level_med);
        // printGrids(low_level, low_level_med, avg_level);

        GroundPlane fine_plane = calcGridLevel(input_cloud,
                                               quarter_level,
                                               dirty_plane,
                                               half_level_med,
                                               distillate_sigma_shift,
                                               +min_correction,
                                               min_points_avg,
                                               max_ground_radius);

        grid_median_recovery(quarter_level, quarter_level_med);

        // printGrids(low_level, avg_level, avg_level_med);

        GroundPlane perfect_plane = calcGridLevel(input_cloud,
                                                  th_level,
                                                  fine_plane,
                                                  quarter_level_med,
                                                  normal_th_sigma_shift,
                                                  +big_correction,
                                                  min_points_high,
                                                  max_ground_radius);

        grid_median_recovery(th_level, th_level_med);

        printf("dirty_plane:   z = %.3f * x + %.3f *y + %.3f\t\t\t", dirty_plane.a, dirty_plane.b, dirty_plane.c);
        printf("fine_plane:    z = %.3f * x + %.3f *y + %.3f\t\t\t", fine_plane.a, fine_plane.b, fine_plane.c);
        printf("perfect_plane: z = %.3f * x + %.3f *y + %.3f\n", perfect_plane.a, perfect_plane.b, perfect_plane.c);

        printGrids(half_level, quarter_level, th_level);
        // printGrids(half_level_med, quarter_level_med, th_level_med);

        applyGridLevel(input_cloud, plain_cloud, other_cloud, perfect_plane,  th_level_med);
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
    const float UNKNOWN_GROUND = FLT_MIN;
    const float EMPTY_GRID = FLT_MAX;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_filter_node");

    PointCloudFilterNode filter_node;

    ros::spin();

    return 0;
}
