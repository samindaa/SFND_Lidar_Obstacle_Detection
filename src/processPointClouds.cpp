// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <Eigen/Dense>
#include <random>
#include <unordered_set>

// constructor:
template <typename PointT> ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT> ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
    Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to do voxel grid point reduction and region
  // based filtering

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  return cloud;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(
    pcl::PointIndices::Ptr inliers,
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  // TODO: Create two new point clouds, one cloud with obstacles and other with
  // segmented plane
  typename pcl::PointCloud<PointT>::Ptr obs_cloud(
      new typename pcl::PointCloud<PointT>()),
      plane_cloud(new typename pcl::PointCloud<PointT>());

  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;
  // Extract the inliers
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*plane_cloud);

  // Create the filtering object
  extract.setNegative(true);
  extract.filter(*obs_cloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult(obs_cloud, plane_cloud);
  return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
    float distanceThreshold) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
  //  pcl::PointIndices::Ptr inliers;
  // TODO:: Fill in this function to find inliers for the cloud.
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);
  seg.setInputCloud(cloud);
  // Segment the largest planar component from the cloud
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0) {
    std::cerr << "Could not estimate a planar mode forl the given dataset."
              << std::endl;
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult = SeparateClouds(inliers, cloud);
  return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Segment(typename pcl::PointCloud<PointT>::Ptr cloud,
                                    int maxIterations,
                                    float distanceThreshold) {
  auto startTime = std::chrono::steady_clock::now();

  if (cloud->points.size() < 3) {
    return {};
  }
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<size_t> dis(0, cloud->points.size() - 1);

  std::unordered_set<size_t> inliers_result;
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

    const Eigen::Vector3f p1(cloud->points[sample_points[0]].x,
                             cloud->points[sample_points[0]].y,
                             cloud->points[sample_points[0]].z);
    const Eigen::Vector3f p2(cloud->points[sample_points[1]].x,
                             cloud->points[sample_points[1]].y,
                             cloud->points[sample_points[1]].z);
    const Eigen::Vector3f p3(cloud->points[sample_points[2]].x,
                             cloud->points[sample_points[2]].y,
                             cloud->points[sample_points[2]].z);
    //    std::cout << "p1: " << p1 << "\np2: " << p2 << "\np3:" << p3 <<
    //    std::endl;

    const Eigen::Vector3f v1 = p2 - p1;
    const Eigen::Vector3f v2 = p3 - p1;
    const Eigen::Vector3f nn = v1.cross(v2);

    const Eigen::Vector4f line_coeff(nn.x(), nn.y(), nn.z(),
                                     -nn.dot(p1)); // ABCD
    const float right_const =
        distanceThreshold * distanceThreshold *
        (line_coeff.x() * line_coeff.x() + line_coeff.y() * line_coeff.y() +
         line_coeff.z() * line_coeff.z());
    std::unordered_set<size_t> result;
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      Eigen::Vector4f point(cloud->points[i].x, cloud->points[i].y,
                            cloud->points[i].z, 1.0F);
      if (std::pow(line_coeff.dot(point), 2) <= right_const) {
        result.insert(i);
      }
    }
    if (result.size() > inliers_result.size()) {
      //      std::cout << "result: " << result.size()
      //                << " inlierResult: " << inliersResult.size() <<
      //                std::endl;
      inliers_result = std::move(result);
    }

  } while (--maxIterations > 0);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  typename pcl::PointCloud<PointT>::Ptr cloud_inliers(
      new pcl::PointCloud<PointT>()),
      cloud_outliers(new pcl::PointCloud<PointT>());

  for (size_t index = 0; index < cloud->points.size(); index++) {
    const pcl::PointXYZ &point = cloud->points[index];
    if (inliers_result.count(index)) {
      cloud_inliers->points.push_back(point);
    } else {
      cloud_outliers->points.push_back(point);
    }
  }
  return {cloud_outliers, cloud_inliers};
}

template <typename PointT>
void ProcessPointClouds<PointT>::proximity(
    typename pcl::PointCloud<PointT>::Ptr cloud, const int &id,
    std::vector<bool> &processed, std::vector<int> &cluster,
    std::shared_ptr<KdTree> tree, float distanceTol) {
  processed[id] = true;
  cluster.emplace_back(id);
  const std::vector<int> nearby_points = tree->search(
      {cloud->points[id].x, cloud->points[id].y, cloud->points[id].z},
      distanceTol);
  for (const auto &nearby_point_id : nearby_points) {
    if (!processed[nearby_point_id]) {
      proximity(cloud, nearby_point_id, processed, cluster, tree, distanceTol);
    }
  }
}

template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclidean_cluster(
    typename pcl::PointCloud<PointT>::Ptr cloud, std::shared_ptr<KdTree> tree,
    float distanceTol) {

  // TODO: Fill out this function to return list of indices for each cluster

  std::vector<std::vector<int>> clusters;
  std::vector<bool> processed(cloud->points.size(), false);
  for (size_t id = 0; id < cloud->points.size(); ++id) {
    if (!processed[id]) {
      std::vector<int> cluster;
      proximity(cloud, id, processed, cluster, tree, distanceTol);
      clusters.emplace_back(cluster);
    }
  }

  return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,
    int minSize, int maxSize) {

  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to perform euclidean clustering to group
  // detected obstacles

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  std::shared_ptr<KdTree> tree = std::make_shared<KdTree>();
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    tree->insert({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z},
                 i);
  }

  // TODO(saminda): use min and max
  std::vector<std::vector<int>> ec_clusters =
      euclidean_cluster(cloud, tree, clusterTolerance);

  std::cout << "clusers: " << clusters.size() << std::endl;

  for (const std::vector<int> &cluster : ec_clusters) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    for (const int &indice : cluster) {
      cluster_cloud->points.push_back(cloud->points[indice]);
    }
    clusters.emplace_back(cluster_cloud);
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count()
            << " milliseconds and found " << clusters.size() << " clusters"
            << std::endl;

  return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(
    typename pcl::PointCloud<PointT>::Ptr cluster) {

  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Box box;
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;

  return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(
    typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file
            << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::loadPcd(std::string file) {

  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file
            << std::endl;

  return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path>
ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {

  std::vector<boost::filesystem::path> paths(
      boost::filesystem::directory_iterator{dataPath},
      boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}
