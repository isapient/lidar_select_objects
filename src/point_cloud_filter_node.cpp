#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/filters/extract_indices.h>
#include <pcl_msgs/ModelCoefficients.h>
// #include <pcl_ros/point_indices.h>
#include <pcl_ros/segmentation/sac_segmentation.h>
#include <pcl_ros/segmentation/extract_clusters.h>
#include <pcl_ros/filters/extract_indices.h>
#include <pcl_ros/filters/statistical_outlier_removal.h>
// #include <pcl_ros/common/time.h>

#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

struct GroundPlane
{
    float a, b, c; // z = a*x + b*y + c
    GroundPlane()
    {
        a = b = c = 0;
    }
};

class Momentums
{
public:
    int cnt;
    float x, y, z; // first order
    float zz;      // second order
    Momentums()
    {
        cnt = 0;
        x = y = z = 0;
        zz = 0;
    }
    void accumulate(float x, float y, float z)
    {
        cnt++;
        this->x += x;
        this->y += y;
        this->z += z;
        this->zz += z * z;
    }
    void finalize()
    {
        if (cnt == 0)
            return;
        x /= cnt;
        y /= cnt;
        z /= cnt;
        zz = zz / cnt - z * z;
    }
    Momentums &operator+=(const Momentums &a)
    {
        cnt += a.cnt;
        x += a.x;
        y += a.y;
        z += a.z;
        zz += a.zz;
        return *this;
    }
};

class PointCloudFilterNode
{
public:
    void applyGridLevel(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &plain_cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &other_cloud,
                        const GroundPlane &plane,
                        const std::vector<float> &underlying_th,
                        const float min_radius,
                        std::vector<float> &range_aperture)
    {
        const float min_radius_sq = min_radius * min_radius;
        range_aperture.resize(_ring_cnt * _ring_len);

        for (int y = 0; y < _map_h; y++)
        {
            for (int x = 0; x < _map_w; x++)
            {
                const int grid_idx = x + y * _map_w;
                const float grid_level = underlying_th[grid_idx];

                if (grid_level == EMPTY_GRID)
                    continue;

                const int x1 = _cell_w * x;
                const int x2 = _cell_w * (x + 1);
                const int y1 = _cell_h * y;
                const int y2 = _cell_h * (y + 1);
                for (int i = x1; i < x2; i++)
                {
                    for (int j = y1; j < y2; j++)
                    {
                        int pulse_idx = i + j * _ring_len;
                        float x = input_cloud->points[pulse_idx].x;
                        float y = input_cloud->points[pulse_idx].y;
                        float z = input_cloud->points[pulse_idx].z;
                        if (z == 0 && x == 0 && y == 0)
                        {
                            range_aperture[pulse_idx] = 0; // EMPTY
                            continue;
                        }

                        float z0 = plane.a * x + plane.b * y + plane.c;

                        if ((z > z0 + grid_level || grid_level == UNKNOWN_GROUND) && x * x + y * y >= min_radius_sq)
                        {
                            range_aperture[pulse_idx] = sqrt(x * x + y * y + z * z);
                            other_cloud->points.push_back(input_cloud->points[i + j * _ring_len]);
                        }
                        else
                        {
                            range_aperture[pulse_idx] = -1; // GROUND
                            plain_cloud->points.push_back(input_cloud->points[i + j * _ring_len]);
                        }
                    }
                }
            }
        }
    }

    void debugPrintGrids(const std::vector<float> &level1, const std::vector<float> &level2, const std::vector<float> &level3)
    {
        float v;
        for (int y = 0; y < _map_h; y++)
        {
            for (int x = 0; x < _map_w; x++)
            {
                v = level1[x + y * _map_w];
                if (v == UNKNOWN_GROUND)
                    printf(" xx ");
                else if (v == EMPTY_GRID)
                    printf(" .. ");
                else
                    printf("%3d ", (int)(10 * v + 0.5));
            }
            printf("| ");
            for (int x = 0; x < _map_w; x++)
            {
                v = level2[x + y * _map_w];
                if (v == UNKNOWN_GROUND)
                    printf(" xx ");
                else if (v == EMPTY_GRID)
                    printf(" .. ");
                else
                    printf("%3d ", (int)(10 * v + 0.5));
            }
            printf("| ");
            for (int x = 0; x < _map_w; x++)
            {
                v = level3[x + y * _map_w];
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

    void median_3x3(const std::vector<float> &src, std::vector<float> &dst, const int cnt_th, bool process_known_only)
    {
        for (int y = 0; y < _map_h; y++)
        {
            for (int x = 0; x < _map_w; x++)
            {
                if (process_known_only == true &&
                    (src[x + y * _map_w] == UNKNOWN_GROUND ||
                     src[x + y * _map_w] == EMPTY_GRID))
                {
                    dst[x + y * _map_w] = src[x + y * _map_w];
                    continue;
                }
                std::vector<float> aperture;
                aperture.reserve(9);
                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        int nx = ((x + dx) + _map_w) % _map_w; // cycled over lidar rings
                        int ny = y + dy;
                        if (ny < 0 || ny >= _map_h)
                            continue;
                        float val = src[nx + ny * _map_w];
                        if (val == EMPTY_GRID or val == UNKNOWN_GROUND)
                            continue;
                        aperture.push_back(val);
                    }
                }
                if (aperture.size() < cnt_th)
                {
                    if (src[x + y * _map_w] == EMPTY_GRID)
                        dst[x + y * _map_w] = EMPTY_GRID;
                    else
                        dst[x + y * _map_w] = UNKNOWN_GROUND;
                }
                else
                {
                    std::sort(aperture.begin(), aperture.end());
                    int size = aperture.size();
                    if (size % 2 == 1)
                    {
                        dst[x + y * _map_w] = aperture[size / 2]; // odd rule (center element)
                    }
                    else
                    {
                        dst[x + y * _map_w] = (aperture[size / 2 - 1] + aperture[size / 2]) / 2; // even rule (average of two center elements)
                    }
                }
            }
        }
    }

    void copy_empty_mask(const std::vector<float> &src, std::vector<float> &dst)
    {
        for (int p = 0; p < _map_h * _map_w; p++)
        {
            if (src[p] == EMPTY_GRID)
            {
                dst[p] = EMPTY_GRID;
            }
        }
    }

    void copy_grid(const std::vector<float> &src, std::vector<float> &dst)
    {
        for (int p = 0; p < _map_h * _map_w; p++)
        {
            dst[p] = src[p];
        }
    }

    /**
     * @brief Median recovery of the grid level
     * input - input grid
     * output - output grid
     */
    void grid_median_recovery(const std::vector<float> &input, std::vector<float> &output)
    {
        const int primary_min_median_neigbours = 3;
        const int secondary_min_median_neigbours = 3;

        std::vector<float> grid_med_1(_map_w * _map_h);
        std::vector<float> grid_med_2(_map_w * _map_h);

        median_3x3(input, grid_med_1, primary_min_median_neigbours, true);
        median_3x3(grid_med_1, grid_med_2, primary_min_median_neigbours, true);

        std::vector<float> grid_med_3(_map_w * _map_h);

        for (int i = 0; i < 16; i++)
        {
            median_3x3(grid_med_2, grid_med_3, secondary_min_median_neigbours, false);
            median_3x3(grid_med_3, grid_med_2, secondary_min_median_neigbours, false);
        }
        copy_empty_mask(input, grid_med_2);
        copy_grid(grid_med_2, output);
    }

    int ransacPlaneModel(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::ModelCoefficients::Ptr &coefficients)
    {
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;

        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);

        // float probability = 0.99;

        seg.setAxis(Eigen::Vector3f(0., 0., 1.)); // Set the vertical axis along which we need to search for a model perpendicular to
        seg.setEpsAngle((7.f * M_PI) / 180.);     // Set maximum allowed difference between the model normal and the given axis in radians
        seg.setOptimizeCoefficients(true);        // Coefficient refinement is required

        seg.setDistanceThreshold(0.10f);
        seg.setMaxIterations(1000);
        // seg.setProbability(probability);

        seg.setInputCloud(input_cloud);
        seg.segment(*inliers, *coefficients);

        return inliers->indices.size();
    }

    GroundPlane calcGridLevel(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                              std::vector<float> &result_grid,
                              const GroundPlane &plane,
                              const std::vector<float> &underlying_th,
                              const float msq_mult_correction,
                              const float min_abs_correction,
                              const int min_points,
                              const float max_ground_radius = 30.f,
                              const float min_ground_radius = 3.0f,
                              const float max_underground_z = 4.0f,
                              const int min_cells = 4)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr ransac_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<Momentums> cell_stats(_map_w * _map_h); // zero initialized
        Momentums stats;

        float max_ground_radius_sq = max_ground_radius * max_ground_radius;
        float min_ground_radius_sq = min_ground_radius * min_ground_radius;

        for (int y = 0; y < _map_h; y++)
        {
            for (int x = 0; x < _map_w; x++)
            {
                const int x1 = _cell_w * x;
                const int x2 = _cell_w * (x + 1);
                const int y1 = _cell_h * y;
                const int y2 = _cell_h * (y + 1);
                const int cell_idx = x + y * _map_w;

                int non_zero_count = 0;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cell_cloud(new pcl::PointCloud<pcl::PointXYZ>);

                if (underlying_th[cell_idx] == EMPTY_GRID)
                {
                    result_grid[cell_idx] = EMPTY_GRID;
                    continue;
                }

                for (int i = x1; i < x2; i++)
                {
                    for (int j = y1; j < y2; j++)
                    {
                        int pulse_idx = i + j * _ring_len;
                        float x = input_cloud->points[pulse_idx].x;
                        float y = input_cloud->points[pulse_idx].y;
                        float z = input_cloud->points[pulse_idx].z;
                        if (z == 0 && x == 0 && y == 0)
                            continue;

                        non_zero_count++;

                        float z0 = plane.a * x + plane.b * y + plane.c;

                        if (z > z0 + underlying_th[cell_idx] && underlying_th[cell_idx] != UNKNOWN_GROUND)
                            continue;

                        if (z < z0 - max_underground_z)
                            continue;

                        const float sq_radius = x * x + y * y;
                        if (sq_radius > max_ground_radius_sq || sq_radius < min_ground_radius_sq)
                            continue;

                        // printf("(%.1f; %.1f; %.1f) ", x, y, z);
                        cell_stats[cell_idx].accumulate(x, y, z);
                        cell_cloud->points.push_back(input_cloud->points[pulse_idx]);
                    }
                }

                if (cell_stats[cell_idx].cnt < min_points)
                {
                    if (non_zero_count != 0)
                        result_grid[cell_idx] = UNKNOWN_GROUND;
                    else
                        result_grid[cell_idx] = EMPTY_GRID;
                }
                else
                {
                    result_grid[cell_idx] = 0; // filler - should be calculated later
                    stats += cell_stats[cell_idx];
                    *ransac_cloud += *cell_cloud; // Concatenate points from cell cloud into ransac_cloud
                }
            }
        }

        // stats.finalize();
        pcl::ModelCoefficients::Ptr ransac_plane(new pcl::ModelCoefficients());

        int inliers_count = ransacPlaneModel(ransac_cloud, ransac_plane);

        GroundPlane fine_plane = plane; // by default do not correct plane

        if (inliers_count > min_points * min_cells)
        {
            // Use the obtained plane coefficients
            const float ransac_A = ransac_plane->values[0];
            const float ransac_B = ransac_plane->values[1];
            const float ransac_C = ransac_plane->values[2];
            const float ransac_D = ransac_plane->values[3];
            const float denominator = ransac_C;
            if (denominator > 0.00001f || denominator < -0.00001f)
            {
                // convert A*x + B*y + C*z + D = 0 to z = a*x + b*y + c
                fine_plane.a = -ransac_A / denominator;
                fine_plane.b = -ransac_B / denominator;
                fine_plane.c = -ransac_D / denominator;
            }
        }

        for (int y = 0; y < _map_h; y++)
        {
            for (int x = 0; x < _map_w; x++)
            {
                int grid_idx = x + y * _map_w;
                if (result_grid[grid_idx] != EMPTY_GRID && result_grid[grid_idx] != UNKNOWN_GROUND)
                {
                    Momentums over_plane = cell_stats[grid_idx];
                    over_plane.z -= fine_plane.a * over_plane.x +
                                    fine_plane.b * over_plane.y +
                                    fine_plane.c * over_plane.cnt;

                    cell_stats[grid_idx].finalize();
                    over_plane.finalize();

                    float msq = sqrt(cell_stats[grid_idx].zz); // original z MSQ
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

    /**
     * @brief Save range aperture as PNG image (pure debug visualization)
     */
    void debugRangeApertureToPNG(const std::vector<float> &range_aperture, const std::string &filename)
    {
        int width = _ring_len;
        int height = _ring_cnt;
        std::vector<unsigned char> pixels(width * height);

        // Normalize and convert range aperture values to unsigned char
        float max_range = *std::max_element(range_aperture.begin(), range_aperture.end());
        if (max_range >= 60.0f)
            max_range = 60.0f;

        for (int i = 0; i < height; ++i)
        {
            for (int j = 0; j < width; ++j)
            {
                float value = range_aperture[j + i * _ring_len];
                if (value < 0) // GROUND pulse
                    pixels[j + i * width] = 25;
                else if (value == 0) // EMPTY pulse
                    pixels[j + i * width] = 0;
                else if (value >= 25.0f) // DUST
                    pixels[j + i * width] = 50;
                else
                    pixels[j + i * width] = 100 * static_cast<unsigned char>(value / 25 * 155.0f);
            }
        }

        // Save as PNG
        stbi_write_png(filename.c_str(), width, height, 1, pixels.data(), width * sizeof(unsigned char));
    }

    void findPlainPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &plain_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &other_cloud)
    {

        std::vector<float> half_level(_map_w * _map_h);
        std::vector<float> half_level_med(_map_w * _map_h);

        std::vector<float> quarter_level(_map_w * _map_h);
        std::vector<float> quarter_level_med(_map_w * _map_h);

        std::vector<float> th_level(_map_w * _map_h);
        std::vector<float> th_level_med(_map_w * _map_h);

        const int min_points_low = 48; // about 36%
        const int min_points_avg = 16;
        const int min_points_high = 8; // about 15%

        const float lowpart_sigma_shift = -0.7;    // keeping ~25% of samples under threshold
        const float distillate_sigma_shift = +0.7; // keeping 75% of remaining samples (18% of total samples)
        const float normal_th_sigma_shift = +2.5;  // more than 3 sigma above average ground level

        const float no_correction = 0.0;   // 0 centimeters
        const float min_correction = 0.10; // add 5 centimeters (about lidar linear accuracy)
        const float big_correction = 0.25; // add more over ground level

        const float rough_ground_radius = 15.0f; // estimate ground in this radius
        const float fine_ground_radius = 20.0f;  // estimate ground in this radius
        const float th_ground_radius = 20.0f;    // estimate ground in this radius

        const float rough_minradius = 3.0f; // longer than dust body size
        const float fine_minradius = 5.0f;  // buggy body size
        const float th_minradius = 5.0f;    // buggy body size

        const float rough_min_underground = 3.0f;
        const float fine_min_underground = 0.5f;
        const float th_min_underground = 0.3f;

        GroundPlane horizontal_plane; // inited by zero coefficients

        std::vector<float> unknown_levels(_map_w * _map_h);
        for (int i = 0; i < _map_h * _map_w; i++)
            unknown_levels[i] = UNKNOWN_GROUND;

        GroundPlane dirty_plane = calcGridLevel(input_cloud,
                                                half_level,
                                                horizontal_plane,
                                                unknown_levels,
                                                lowpart_sigma_shift,
                                                no_correction,
                                                min_points_low,
                                                rough_ground_radius,
                                                rough_minradius,
                                                rough_min_underground);

        // printGrids(low_level, low_level, low_level);        // DEBUG preliminary elevation levels filtration

        grid_median_recovery(half_level, half_level_med);
        // printGrids(low_level, low_level_med, avg_level);    // DEBUG intermediate elevation levels extraction

        GroundPlane fine_plane = calcGridLevel(input_cloud,
                                               quarter_level,
                                               dirty_plane,
                                               half_level_med,
                                               distillate_sigma_shift,
                                               +min_correction,
                                               min_points_avg,
                                               fine_ground_radius,
                                               fine_minradius,
                                               fine_min_underground);

        grid_median_recovery(quarter_level, quarter_level_med);

        // printGrids(low_level, avg_level, avg_level_med);     // DEBUG intermediate elevation levels filtration

        GroundPlane perfect_plane = calcGridLevel(input_cloud,
                                                  th_level,
                                                  fine_plane,
                                                  quarter_level_med,
                                                  normal_th_sigma_shift,
                                                  +big_correction,
                                                  min_points_high,
                                                  th_ground_radius,
                                                  th_minradius,
                                                  th_min_underground);

        grid_median_recovery(th_level, th_level_med);

        // printf("dirty_plane:   z = %.3f * x + %.3f *y + %.3f\t\t\t", dirty_plane.a, dirty_plane.b, dirty_plane.c);
        // printf("fine_plane:    z = %.3f * x + %.3f *y + %.3f\t\t\t", fine_plane.a, fine_plane.b, fine_plane.c);
        // printf("perfect_plane: z = %.3f * x + %.3f *y + %.3f\n", perfect_plane.a, perfect_plane.b, perfect_plane.c);

        // debugPrintGrids(half_level, quarter_level, th_level); // DEBUG scanned elevation levels 
        // printGrids(half_level_med, quarter_level_med, th_level_med); // DEBUG filtered elevation levels

        std::vector<float> range_aperture;
        applyGridLevel(input_cloud, plain_cloud, other_cloud,
                       perfect_plane, th_level_med,
                       th_minradius,
                       range_aperture);

        // PNG DEBUG the raw LiDARrange aperture with thresholds and classes applied
        // std::string filename = "/home/isap/dbg/range_aperture_" + std::to_string(10000 + frame_count) + ".png";
        // printf("%s ", filename.c_str());
        // saveRangeApertureAsPNG(range_aperture, filename);

        frame_count++;
        if(frame_count % 10 == 0) printf(" lidar frame count:  %d\n", frame_count);
    }

    // Cluster the point cloud and separate the objects from the dust by penetrability
    //
    void clusterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &object_cloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &rest_cloud)
    {
        int min_cluster_size = 9;                                // Minimum number of points in a cluster 9 is okay both for 128 and 64 rings lidars
        float cluster_tolerance = 4.5;                           // Cluster tolerance: 3 meters is enough for long range buildings (and this is actually baggy size)
        bool low_res_metrics = (_ring_cnt <= 64) ? true : false; // Low resolution metrics for 64 rings lidar

        // Create a k-d tree for fast nearest neighbor searches
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>);
        kd_tree->setInputCloud(input_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        std::vector<bool> processed(input_cloud->size(), false);

        // Iterate through each point in the input cloud
        for (std::size_t i = 0; i < input_cloud->size(); ++i)
        {
            if (!processed[i])
            {
                // Search for neighboring points within the cluster tolerance
                std::vector<int> cluster_indices_vec;
                std::vector<float> distances;
                pcl::PointXYZ search_point = input_cloud->points[i];
                kd_tree->radiusSearch(search_point, cluster_tolerance, cluster_indices_vec, distances);

                // Check if the cluster size is greater than the minimum
                if (cluster_indices_vec.size() >= min_cluster_size)
                {
                    pcl::PointIndices indices;
                    for (std::size_t j = 0; j < cluster_indices_vec.size(); ++j)
                    {
                        if (!processed[cluster_indices_vec[j]])
                        {
                            indices.indices.push_back(cluster_indices_vec[j]);
                            processed[cluster_indices_vec[j]] = true;
                        }
                    }
                    cluster_indices.push_back(indices);
                }
            }
        }

        // printf("%ld clusters found\n", cluster_indices.size());
        int cluster_num = 0;

        // Analyze the clusters properties: penetrability, chaoticity, and sparseness
        for (const auto &indices : cluster_indices)
        {
            cluster_num++;
            pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
            float object_range = 0;
            int points_in_object = 0;
            for (const auto &index : indices.indices)
            {
                const pcl::PointXYZ point = input_cloud->points[index];
                if (point.x == 0 && point.y == 0 && point.z == 0)
                    continue;
                points_in_object++;
                object_range += sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
                object->points.push_back(point);
            }
            if (points_in_object > 0)
                object_range /= points_in_object;

            object->width = points_in_object;
            object->height = 1;
            object->is_dense = true;

            if (points_in_object < min_cluster_size)
            {
                *rest_cloud += *object;

                // for (auto &point : object->points) // DEBUG VISUALIZATION
                //     point.z = 30;                  // DEBUG by observing altitude of objects
                // *object_cloud += *object;          // DEBUG

                continue;
            }

            // project the object to the sphere to find nearest LiDAR pulses
            pcl::PointCloud<pcl::PointXYZ>::Ptr projection(new pcl::PointCloud<pcl::PointXYZ>);
            for (auto &original_point : object->points)
            {
                pcl::PointXYZ point = original_point;
                const float range = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
                const float mult = object_range / range;
                point.x *= mult;
                point.y *= mult;
                point.z *= mult;

                projection->points.push_back(point);
            }

            const float penetrability_threshold = low_res_metrics ? 7 : 17;  // reject most of cloud, save the trees
            const float chaoticity_threshold = 1.6;                          // reject non regular structures (dust, partially scanned objects)
            const float sparseness_threshold = low_res_metrics ? 13.0 : 7.1; // reject too sparse cluster (dust, partially scanned objects)

            const int neighbor_count = 7; // basic neighbour count for the analysis

            float penetrability = 0.0; // point variation along the lidar rays
            float sparseness = 0.0;    // relative average interpoint distance
            float chaoticity = 0.0;    // variation of point distance distribution skewness

            float skewness_m1 = 0.0;
            float skewness_m2 = 0.0;

            // Build KdTree
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_proj;
            kdtree_proj.setInputCloud(projection);
            // Iterate through each point in the point cloud
            for (size_t i = 0; i < projection->points.size(); ++i)
            {
                std::vector<int> nn_indices(neighbor_count + 1);      // Store indices of nearest neighbors (excluding the point itself)
                std::vector<float> nn_proj_dists(neighbor_count + 1); // Store distances to nearest neighbors in "projection"
                std::vector<float> nn_orig_dists(neighbor_count + 1); // Store distances to nearest neighbors

                // Perform nearest neighbor search
                kdtree_proj.nearestKSearch(projection->points[i], neighbor_count, nn_indices, nn_proj_dists);

                // Retrieve corresponding points from the original object point cloud
                int count = 0;
                float point_jog = 0;
                float neighbor_relative_distance = 0;
                float avg_distance = 0;

                for (size_t j = 0; j < nn_indices.size(); ++j)
                {
                    int nn_index = nn_indices[j];
                    if (nn_index == i)
                        continue; // do not compare with itself
                    count++;

                    pcl::PointXYZ original_point = object->points[i];  // original analyzing point
                    pcl::PointXYZ nn_point = object->points[nn_index]; // corresponding nearest neighbor point

                    // Calculate the distance between the points
                    nn_orig_dists[j] = sqrt(pow(original_point.x - nn_point.x, 2) +
                                            pow(original_point.y - nn_point.y, 2) +
                                            pow(original_point.z - nn_point.z, 2));

                    // printf("%.0f;%.0f;  ", nn_orig_dists[j]*100, nn_proj_dists[j]*100);

                    point_jog += pow(nn_orig_dists[j] / nn_proj_dists[j], 2) - 1;
                    // point_jog += nn_orig_dists[j] / nn_proj_dists[j];
                    neighbor_relative_distance += pow(nn_orig_dists[j], 2);
                    avg_distance += nn_orig_dists[j];
                }

                // penetrability and sparseness
                if (count > 0)
                {
                    point_jog /= count;
                    if (point_jog >= 0)
                    {
                        penetrability += sqrt(point_jog);
                        // penetrability += point_jog;
                    }

                    neighbor_relative_distance /= count;
                    if (neighbor_relative_distance >= 0)
                    {
                        sparseness += sqrt(neighbor_relative_distance);
                    }

                    avg_distance /= count;
                }

                // Calculate chaoicity

                float upper_avg_distance = 0;
                float lower_avg_distance = 0;
                int upper_avg_count = 0;
                int lower_avg_count = 0;

                for (size_t j = 0; j < nn_indices.size(); ++j)
                {
                    int nn_index = nn_indices[j];
                    if (nn_index == i)
                        continue; // do not compare with itself
                    if (nn_orig_dists[j] > avg_distance)
                    {
                        upper_avg_distance += nn_orig_dists[j];
                        upper_avg_count++;
                    }
                    else
                    {

                        lower_avg_distance += nn_orig_dists[j];
                        lower_avg_count++;
                    }
                }
                if (upper_avg_count > 0)
                    upper_avg_distance /= upper_avg_count;
                else
                    upper_avg_distance = FLT_MAX;

                if (lower_avg_count > 0)
                    lower_avg_distance /= lower_avg_count;
                else
                    lower_avg_distance = FLT_MIN;

                float skewness = (lower_avg_distance / upper_avg_distance);

                skewness_m1 += skewness;
                skewness_m2 += skewness * skewness;
            }

            const int size = projection->points.size();

            if (size > 0)
                penetrability /= size;
            else
                penetrability = FLT_MAX;

            if (size > 0 && object_range > 0.00001f)
            {
                sparseness /= object_range * size;
                sparseness *= 100; // percentage
            }
            else
                sparseness = FLT_MAX;

            if (size > 0)
            {
                skewness_m1 /= size;
                skewness_m2 /= size;
                chaoticity = sqrt(skewness_m2 - skewness_m1 * skewness_m1);
                chaoticity *= 10; // normalize to ~10
            }
            else
            {
                chaoticity = FLT_MAX;
            }

            // printf("%.1f  ", penetrability);
            // printf("%.1f  ", sparseness);
            // printf("%.1f  ", chaoticity);

            if (penetrability < penetrability_threshold &&
                chaoticity < chaoticity_threshold &&
                sparseness < sparseness_threshold)
            {
                *object_cloud += *object;

                // for (auto &point : projection->points) // DEBUG VISUALIZATION
                // {
                //     // point.z = 30 + chaoticity * 20; // DEBUG by observing altitude of objects
                //     // point.z = 30 + penetrability * 4; // DEBUG by observing altitude of objects
                //     point.z = 30 + sparseness * 5; // DEBUG by observing altitude of objects
                // }
                // *object_cloud += *projection; // DEBUG
            }
            else
            {
                *rest_cloud += *object;

                // for (auto &point : projection->points) // DEBUG VISUALIZATION
                // {
                //     // point.z = 30 + chaoticity * 20; // DEBUG by observing altitude of objects
                //     // point.z = 30 + penetrability * 4; // DEBUG by observing altitude of objects
                //     point.z = 30 + sparseness * 5; // DEBUG by observing altitude of objects
                // }
                // *rest_cloud += *projection; // DEBUG
            }
        }
        // printf("\n");

        // Populate the rest of the points
        for (std::size_t i = 0; i < input_cloud->size(); ++i)
        {
            if (!processed[i])
            {
                rest_cloud->points.push_back(input_cloud->points[i]);
            }
        }
        rest_cloud->width = rest_cloud->points.size();
        rest_cloud->height = 1;
        rest_cloud->is_dense = true;
    }

    // Constructor for custom topic
    PointCloudFilterNode(const std::string &input_topic) : _input_topic(input_topic)
    {
        // Subscribe to the input point cloud topic
        _input_cloud_sub = _nh.subscribe<sensor_msgs::PointCloud2>(_input_topic, 1,
                                                                   &PointCloudFilterNode::pointCloudCallback, this);

        printf("lidar_filter_objects subscribed to topic: %s\n", _input_topic.c_str());

        // Create publishers for separated point clouds
        _object_cloud_pub = _nh.advertise<sensor_msgs::PointCloud2>("/lidar_filter/signal", 1);
        // _ground_cloud_pub = _nh.advertise<sensor_msgs::PointCloud2>("/lidar_filter/ground", 1);
        _noise_cloud_pub = _nh.advertise<sensor_msgs::PointCloud2>("/lidar_filter/noise", 1);

        printf("lidar_filter_objects publishing topic: %s\n", "/lidar_filter/signal");
        printf("lidar_filter_objects publishing topic: %s\n", "/lidar_filter/noise");

        // Set filtering parameters
        //
    }

    // Callback function for the input point cloud implementing two steps: ground filtering and object separation
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &input_cloud)
    {
        // Convert ROS point cloud to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input_cloud, *pcl_cloud);

        _ring_cnt = input_cloud->height; // 64 for os1, 128 for os2 and os3
        _ring_len = input_cloud->width;

        // printf("ring_cnt = %d, ring_len = %d\n", _ring_cnt, _ring_len);

        // Create separate point cloud containers for objects and non-objects
        pcl::PointCloud<pcl::PointXYZ>::Ptr work_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Separate plain points from other cloud
        _cell_h = _ring_cnt / _map_h; // = 4
        _cell_w = _ring_len / _map_w; // = 32

        // printf("v_block = %d, h_block = %d\n", _cell_h, _cell_w);

        findPlainPoints(pcl_cloud, ground_cloud, work_cloud);

        // Create separate point cloud containers for plain and other points
        pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr noise_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Use the segmentation and extraction method
        clusterPointCloud(work_cloud, object_cloud, noise_cloud);


        // Publish the separated point clouds as ROS messages
        sensor_msgs::PointCloud2 object_ros_cloud;
        sensor_msgs::PointCloud2 noise_ros_cloud;
        // sensor_msgs::PointCloud2 ground_ros_cloud;

        *noise_cloud += *ground_cloud;

        pcl::toROSMsg(*object_cloud, object_ros_cloud);
        pcl::toROSMsg(*noise_cloud, noise_ros_cloud);
        // pcl::toROSMsg(*ground_cloud, ground_ros_cloud);

        // Set the header information for ROS messages
        object_ros_cloud.header = input_cloud->header;
        noise_ros_cloud.header = input_cloud->header;
        // ground_ros_cloud.header = input_cloud->header;

        // Publish the separated point clouds
        _object_cloud_pub.publish(object_ros_cloud);
        _noise_cloud_pub.publish(noise_ros_cloud);
        // _ground_cloud_pub.publish(ground_ros_cloud);
    }

private:
    ros::NodeHandle _nh;
    ros::Subscriber _input_cloud_sub;
    ros::Publisher _noise_cloud_pub;
    ros::Publisher _object_cloud_pub;
    ros::Publisher _ground_cloud_pub;
    int frame_count = 0;
    std::string _input_topic = "/mbuggy/os3/points";

    // Input lidar aperture resolution
    int _ring_cnt; // 64 for os1, 128 for os2 and os3
    int _ring_len;

    // ground level scanning blocks
    const int _map_h = 32;
    const int _map_w = 16;

    // scanning cell size
    int _cell_h;
    int _cell_w;

    // scanning cell constants
    static constexpr float UNKNOWN_GROUND = (-FLT_MAX);
    static constexpr float EMPTY_GRID = FLT_MAX;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_filter_node");

    // Initialize ROS node handle
    ros::NodeHandle nh("~"); // Private node handle for node-level parameters

    // Read the input topic parameter from the ROS parameter server
    std::string input_topic;
    if (!nh.getParam("input_topic", input_topic))
    {
        ROS_WARN("Failed to read input topic name from parameter server. Using default value: %s", input_topic.c_str());
        input_topic = "/mbuggy/os3/points"; // Default value
    }

    // Create an instance of the PointCloudFilterNode with the specified input topic
    PointCloudFilterNode filter_node(input_topic);

    ros::spin();

    return 0;
}