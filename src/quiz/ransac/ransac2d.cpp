/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../processPointClouds.h"
#include "../../render/render.h"
#include <Eigen/Dense>
#include <random>
#include <unordered_set>
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  // Add inliers
  float scatter = 0.6;
  for (int i = -5; i < 5; i++) {
    double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = i + scatter * rx;
    point.y = i + scatter * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  // Add outliers
  int numOutliers = 10;
  while (numOutliers--) {
    double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = 5 * rx;
    point.y = 5 * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;

  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  //  return
  //  pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
  return pointProcessor.loadPcd(
      "/home/saminda/Projects/SFND_Lidar_Obstacle_Detection/src/sensors/data/"
      "pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene() {
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("2D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  viewer->addCoordinateSystem(1.0);
  return viewer;
}

std::unordered_set<int>
RansacLineImpl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations,
               float distanceTol) {

  std::unordered_set<int> inliersResult;
  //	srand(time(NULL));

  // TODO: Fill in this function
  if (cloud->points.size() < 2) {
    return inliersResult;
  }
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<size_t> dis(0, cloud->points.size() - 1);

  // For max iterations
  do {
    //    std::cout << "iter: " << maxIterations << std::endl;
    std::vector<size_t> sample_points;
    // Assuming all the sampling points are different by a delta.
    while (sample_points.size() < 2) {
      auto point_idx = dis(gen);
      if (sample_points.empty() || (*sample_points.begin() != point_idx)) {
        sample_points.emplace_back(point_idx);
      }
    }

    Eigen::Vector2d p1(cloud->points[sample_points[0]].x,
                       cloud->points[sample_points[0]].y);
    Eigen::Vector2d p2(cloud->points[sample_points[1]].x,
                       cloud->points[sample_points[1]].y);

    //    std::cout << "p1: " << p1 << " p2: " << p2 << std::endl;

    Eigen::Vector3d line_coeff(p1.y() - p2.y(), p2.x() - p1.x(),
                               p1.x() * p2.y() - p2.x() * p1.y()); // (A, B, C)
    const auto denom = std::sqrt(line_coeff.x() * line_coeff.x() +
                                 line_coeff.y() * line_coeff.y());
    std::unordered_set<int> result;
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      Eigen::Vector3d point(cloud->points[i].x, cloud->points[i].y, 1.0F);
      const auto d = std::fabs(line_coeff.dot(point)) / denom;
      if (d <= distanceTol) {
        result.insert(i);
      }
    }
    if (result.size() > inliersResult.size()) {
      //      std::cout << "result: " << result.size()
      //                << " inlierResult: " << inliersResult.size() <<
      //                std::endl;
      inliersResult = std::move(result);
      //        inliersResult.clear();
      //      inliersResult.insert(result.begin(), result.end());
    }

  } while (--maxIterations > 0);

  // Randomly sample subset and fit line

  // Measure distance between every point and fitted line
  // If distance is smaller than threshold count it as inlier

  // Return indicies of inliers from fitted line with most inliers

  return inliersResult;
}

std::unordered_set<int>
RansacPlaneImpl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations,
                float distanceTol) {
  std::unordered_set<int> inliersResult;
  //	srand(time(NULL));

  // TODO: Fill in this function
  if (cloud->points.size() < 3) {
    return inliersResult;
  }
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<size_t> dis(0, cloud->points.size() - 1);

  // For max iterations
  do {
    //    std::cout << "iter: " << maxIterations << std::endl;
    std::vector<size_t> sample_points;
    // Assuming all the sampling points are different by a delta.
    while (sample_points.size() < 3) {
      auto point_idx = dis(gen);
      bool safe_point_idx = true;
      for (const auto &sample_point : sample_points) {
        if (sample_point == point_idx) {
          safe_point_idx = false;
          break;
        }
      }
      if (safe_point_idx) {
        sample_points.emplace_back(point_idx);
      }
    }

    Eigen::Vector3d p1(cloud->points[sample_points[0]].x,
                       cloud->points[sample_points[0]].y,
                       cloud->points[sample_points[0]].z);
    Eigen::Vector3d p2(cloud->points[sample_points[1]].x,
                       cloud->points[sample_points[1]].y,
                       cloud->points[sample_points[1]].z);
    Eigen::Vector3d p3(cloud->points[sample_points[2]].x,
                       cloud->points[sample_points[2]].y,
                       cloud->points[sample_points[2]].z);
    //    std::cout << "p1: " << p1 << "\np2: " << p2 << "\np3:" << p3 <<
    //    std::endl;

    Eigen::Vector3d v1 = p2 - p1;
    Eigen::Vector3d v2 = p3 - p1;
    Eigen::Vector3d nn = v1.cross(v2);

    Eigen::Vector4d line_coeff(nn.x(), nn.y(), nn.z(), -nn.dot(p1)); // ABCD
    const auto denom = std::sqrt(line_coeff.x() * line_coeff.x() +
                                 line_coeff.y() * line_coeff.y() +
                                 line_coeff.z() * line_coeff.z());
    std::unordered_set<int> result;
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      Eigen::Vector4d point(cloud->points[i].x, cloud->points[i].y,
                            cloud->points[i].z, 1.0F);
      const auto d = std::fabs(line_coeff.dot(point)) / denom;
      if (d <= distanceTol) {
        result.insert(i);
      }
    }
    if (result.size() > inliersResult.size()) {
      //      std::cout << "result: " << result.size()
      //                << " inlierResult: " << inliersResult.size() <<
      //                std::endl;
      inliersResult.clear();
      inliersResult.insert(result.begin(), result.end());
    }

  } while (--maxIterations > 0);

  // Randomly sample subset and fit line

  // Measure distance between every point and fitted line
  // If distance is smaller than threshold count it as inlier

  // Return indicies of inliers from fitted line with most inliers

  return inliersResult;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               int maxIterations, float distanceTol) {
  //  return RansacLineImpl(cloud, maxIterations, distanceTol);
  return RansacPlaneImpl(cloud, maxIterations, distanceTol);
}

int main() {

  // Create viewer
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

  // Create data
  //  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

  // TODO: Change the max iteration and distance tolerance arguments for Ransac
  // function
  std::unordered_set<int> inliers = Ransac(cloud, 20, 0.25);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(
      new pcl::PointCloud<pcl::PointXYZ>());

  for (int index = 0; index < cloud->points.size(); index++) {
    pcl::PointXYZ point = cloud->points[index];
    if (inliers.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }

  // Render 2D point cloud with inliers and outliers
  if (inliers.size()) {
    renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
    renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
  } else {
    renderPointCloud(viewer, cloud, "data");
  }

  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }
}
