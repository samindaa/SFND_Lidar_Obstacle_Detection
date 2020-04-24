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
  typename pcl::PointCloud<PointT>::Ptr filtered_cloud(
      new typename pcl::PointCloud<PointT>());
  pcl::VoxelGrid<PointT> vox_grid;
  vox_grid.setInputCloud(cloud);
  vox_grid.setLeafSize(filterRes, filterRes, filterRes);
  vox_grid.filter(*filtered_cloud);

  typename pcl::PointCloud<PointT>::Ptr cropped_cloud(
      new typename pcl::PointCloud<PointT>());
  pcl::CropBox<PointT> region(true);
  region.setMin(minPoint);
  region.setMax(maxPoint);
  region.setInputCloud(filtered_cloud);
  region.filter(*cropped_cloud);

  // Remove the roof (from notes)
  std::vector<int> roof_indices;
  pcl::CropBox<PointT> roof_region(true);
  region.setMin({-1.5F, -1.7F, -1.0F, 1.0F});
  region.setMax({2.6F, 1.7F, -0.4F, 1.0F});
  region.setInputCloud(cropped_cloud);
  region.filter(roof_indices);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  for (const auto &index : roof_indices) {
    inliers->indices.emplace_back(index);
  }

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cropped_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cropped_cloud);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  return cropped_cloud;
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
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_int_distribution<size_t> dis(0, cloud->points.size() - 1);

  std::vector<int> inliers_result;
  // For max iterations
  while (--maxIterations > 0) {
    //    std::cout << "iter: " << maxIterations << std::endl;
    std::unordered_set<int> sample_points;
    // Assuming all the sampling points are different by a delta.
    while (sample_points.size() < 3) {
      sample_points.insert(dis(gen));
    }

    auto sample_points_iter = sample_points.begin();
    const Eigen::Vector3f p1(cloud->points[*sample_points_iter].x,
                             cloud->points[*sample_points_iter].y,
                             cloud->points[*sample_points_iter].z);
    ++sample_points_iter;
    const Eigen::Vector3f p2(cloud->points[*sample_points_iter].x,
                             cloud->points[*sample_points_iter].y,
                             cloud->points[*sample_points_iter].z);
    ++sample_points_iter;
    const Eigen::Vector3f p3(cloud->points[*sample_points_iter].x,
                             cloud->points[*sample_points_iter].y,
                             cloud->points[*sample_points_iter].z);
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
    std::vector<int> result;
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      Eigen::Vector4f point(cloud->points[i].x, cloud->points[i].y,
                            cloud->points[i].z, 1.0F);
      if (std::pow(line_coeff.dot(point), 2) <= right_const) {
        result.emplace_back(i);
      }
    }
    if (result.size() > inliers_result.size()) {
      //      std::cout << "result: " << result.size()
      //                << " inlierResult: " << inliersResult.size() <<
      //                std::endl;
      inliers_result = std::move(result);
    }

  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  inliers->indices = std::move(inliers_result);
  return SeparateClouds(inliers, cloud);
}

template <typename PointT>
void ProcessPointClouds<PointT>::Proximity(
    typename pcl::PointCloud<PointT>::Ptr cloud, const int &id,
    std::vector<bool> &processed, std::vector<int> &cluster,
    std::shared_ptr<KdTree> tree, float distanceTol, int maxSize) {
  processed[id] = true;
  cluster.emplace_back(id);
  if (cluster.size() >= maxSize) {
    return;
  }
  const std::vector<int> nearby_points = tree->search(
      {cloud->points[id].x, cloud->points[id].y, cloud->points[id].z},
      distanceTol);
  for (const auto &nearby_point_id : nearby_points) {
    if (!processed[nearby_point_id]) {
      Proximity(cloud, nearby_point_id, processed, cluster, tree, distanceTol,
                maxSize);
    }
  }
}

template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::EuclideanCluster(
    typename pcl::PointCloud<PointT>::Ptr cloud, std::shared_ptr<KdTree> tree,
    float distanceTol, int maxSize) {

  // TODO: Fill out this function to return list of indices for each cluster

  std::vector<std::vector<int>> clusters;
  std::vector<bool> processed(cloud->points.size(), false);
  for (size_t id = 0; id < cloud->points.size(); ++id) {
    if (!processed[id]) {
      std::vector<int> cluster;
      Proximity(cloud, id, processed, cluster, tree, distanceTol, maxSize);
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

  std::vector<std::vector<int>> ec_clusters =
      EuclideanCluster(cloud, tree, clusterTolerance, maxSize);

  for (const std::vector<int> &cluster : ec_clusters) {
    if (cluster.size() < minSize) {
      continue;
    }
    typename pcl::PointCloud<PointT>::Ptr cluster_cloud(
        new typename pcl::PointCloud<PointT>());
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
