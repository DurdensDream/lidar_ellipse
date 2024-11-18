#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <vector>
#include <queue>
#include <Eigen/Dense>

// Custom structure for elliptical kernel parameters
struct EllipticalKernel {
    double major_axis;
    double minor_axis;
    double orientation; // in radians
};

class EllipticalDBSCAN {
private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    std::vector<int> labels;
    EllipticalKernel kernel;
    double eps;
    int min_points;
    int current_cluster;

    bool isPointInEllipse(const pcl::PointXYZI& center, const pcl::PointXYZI& point) {
        // Transform point to ellipse local coordinates
        double dx = point.x - center.x;
        double dy = point.y - center.y;
        
        // Rotate point
        double cos_theta = std::cos(kernel.orientation);
        double sin_theta = std::sin(kernel.orientation);
        double x_rot = dx * cos_theta + dy * sin_theta;
        double y_rot = -dx * sin_theta + dy * cos_theta;
        
        // Check if point is inside ellipse
        return ((x_rot * x_rot) / (kernel.major_axis * kernel.major_axis) + 
                (y_rot * y_rot) / (kernel.minor_axis * kernel.minor_axis)) <= 1.0;
    }

    std::vector<int> findNeighbors(int point_idx) {
        std::vector<int> neighbors;
        pcl::PointXYZI center_point = cloud->points[point_idx];
        
        for (int i = 0; i < cloud->size(); ++i) {
            if (i != point_idx && isPointInEllipse(center_point, cloud->points[i])) {
                neighbors.push_back(i);
            }
        }
        return neighbors;
    }

    void expandCluster(int point_idx, std::vector<int>& neighbors) {
        labels[point_idx] = current_cluster;
        
        for (size_t i = 0; i < neighbors.size(); ++i) {
            int neighbor_idx = neighbors[i];
            
            if (labels[neighbor_idx] == -1) { // Unvisited point
                labels[neighbor_idx] = current_cluster;
                
                std::vector<int> new_neighbors = findNeighbors(neighbor_idx);
                if (new_neighbors.size() >= min_points) {
                    neighbors.insert(neighbors.end(), new_neighbors.begin(), new_neighbors.end());
                }
            }
            else if (labels[neighbor_idx] == 0) { // Noise point
                labels[neighbor_idx] = current_cluster;
            }
        }
    }

public:
    EllipticalDBSCAN(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud, 
                     double epsilon, int min_pts, 
                     double major_axis, double minor_axis, double orientation) 
        : cloud(input_cloud), eps(epsilon), min_points(min_pts) {
        labels.resize(cloud->size(), -1); // -1: unvisited, 0: noise, >0: cluster ID
        current_cluster = 0;
        kernel = {major_axis, minor_axis, orientation};
    }

    std::vector<int> getClusters() {
        current_cluster = 0;

        for (int i = 0; i < cloud->size(); ++i) {
            if (labels[i] != -1) continue;

            std::vector<int> neighbors = findNeighbors(i);
            
            if (neighbors.size() < min_points) {
                labels[i] = 0; // Mark as noise
            } else {
                current_cluster++;
                expandCluster(i, neighbors);
            }
        }
        return labels;
    }
};

// Function to visualize clusters with 3D bounding boxes
void visualizeClusters(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, 
                      const std::vector<int>& cluster_labels) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    // Generate random colors for clusters
    std::vector<std::vector<double>> colors;
    int max_cluster = *std::max_element(cluster_labels.begin(), cluster_labels.end());
    for (int i = 0; i <= max_cluster; ++i) {
        colors.push_back({rand() % 255 / 255.0, rand() % 255 / 255.0, rand() % 255 / 255.0});
    }

    // Create colored point clouds for each cluster
    for (int cluster_id = 1; cluster_id <= max_cluster; ++cluster_id) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        
        for (size_t i = 0; i < cluster_labels.size(); ++i) {
            if (cluster_labels[i] == cluster_id) {
                cluster_cloud->points.push_back(cloud->points[i]);
            }
        }
        
        if (cluster_cloud->empty()) continue;

        // Compute cluster bounds
        pcl::PointXYZI min_pt, max_pt;
        pcl::getMinMax3D(*cluster_cloud, min_pt, max_pt);

        // Add bounding box
        viewer->addCube(min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z,
                       colors[cluster_id][0], colors[cluster_id][1], colors[cluster_id][2],
                       "cube_" + std::to_string(cluster_id));
        
        // Add points
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color_handler(
            cluster_cloud, colors[cluster_id][0]*255, colors[cluster_id][1]*255, colors[cluster_id][2]*255);
        viewer->addPointCloud(cluster_cloud, color_handler, "cluster_" + std::to_string(cluster_id));
    }

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <input.pcd>" << std::endl;
        return -1;
    }

    // Load point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(argv[1], *cloud) == -1) {
        std::cerr << "Couldn't read file " << argv[1] << std::endl;
        return -1;
    }

    // Configure DBSCAN parameters
    double eps = 0.5;  // Epsilon neighborhood size
    int min_points = 10;  // Minimum points for core point
    double major_axis = 0.8;  // Major axis of elliptical kernel
    double minor_axis = 0.4;  // Minor axis of elliptical kernel
    double orientation = M_PI / 4;  // 45 degrees rotation

    // Run DBSCAN clustering
    EllipticalDBSCAN dbscan(cloud, eps, min_points, major_axis, minor_axis, orientation);
    std::vector<int> cluster_labels = dbscan.getClusters();

    // Visualize results
    visualizeClusters(cloud, cluster_labels);

    return 0;
}