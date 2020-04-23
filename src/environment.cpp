/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "processPointClouds.h"
#include "render/render.h"
#include "sensors/lidar.h"
#include <memory>
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene,
                             pcl::visualization::PCLVisualizer::Ptr &viewer) {

  Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
  Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
  Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
  Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);

  if (renderScene) {
    renderHighway(viewer);
    egoCar.render(viewer);
    car1.render(viewer);
    car2.render(viewer);
    car3.render(viewer);
  }

  return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display simple highway -----
  // ----------------------------------------------------

  // RENDER OPTIONS
  bool renderScene = true;
  std::vector<Car> cars = initHighway(renderScene, viewer);

  // TODO:: Create lidar sensor
  std::shared_ptr<Lidar> lidar =
      std::make_shared<Lidar>(cars, /*groundPlane=*/0.0);
  auto lidar_scan = lidar->scan();
  //  renderRays(viewer, lidar->position, lidar_scan);
  //  renderPointCloud(viewer, lidar_scan, "scan", Color(1, 0, 0));
  // TODO:: Create point processor
  std::shared_ptr<ProcessPointClouds<pcl::PointXYZ>> point_processor =
      std::make_shared<ProcessPointClouds<pcl::PointXYZ>>();
  //  auto segmented_pair = point_processor->SegmentPlane(lidar_scan, 50, 0.25);
  auto segmented_pair = point_processor->Segment(lidar_scan, 20, 0.25);
  renderPointCloud(viewer, segmented_pair.first, "obs", Color(1, 0, 0));
  renderPointCloud(viewer, segmented_pair.second, "plane", Color(0, 1, 0));

  auto clusters = point_processor->Clustering(segmented_pair.first, 2.0, 3, 60);
  for (size_t cluster_id = 0; cluster_id < clusters.size(); ++cluster_id) {
    Box box = point_processor->BoundingBox(clusters[cluster_id]);
    renderBox(viewer, box, cluster_id);
  }
}

void cityBlock(
    pcl::visualization::PCLVisualizer::Ptr &viewer,
    std::shared_ptr<ProcessPointClouds<pcl::PointXYZI>> point_processor,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud) {

  auto output_cloud = point_processor->FilterCloud(input_cloud, 0.2F,
                                                   {-10.0F, -5.0F, -2.0F, 1.0F},
                                                   {40.0F, 7.0F, 1.0F, 1.0F});

  auto segmented_pair = point_processor->Segment(output_cloud, 20, 0.2);
  auto clusters =
      point_processor->Clustering(segmented_pair.first, 0.6, 15, 650);
  static std::vector<Color> colors = {Color(1, 0, 0), Color(0, 0, 1),
                                      Color(0, 1, 1)};
  for (size_t cluster_id = 0; cluster_id < clusters.size(); ++cluster_id) {
    renderPointCloud(viewer, clusters[cluster_id],
                     "obs" + std::to_string(cluster_id),
                     colors[cluster_id % colors.size()]);
    Box box = point_processor->BoundingBox(clusters[cluster_id]);
    renderBox(viewer, box, cluster_id);
  }

  renderPointCloud(viewer, segmented_pair.second, "plane", Color(0.3, 0.5, 0));
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle,
                pcl::visualization::PCLVisualizer::Ptr &viewer) {

  viewer->setBackgroundColor(0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;

  switch (setAngle) {
  case XY:
    viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
    break;
  case TopDown:
    viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
    break;
  case Side:
    viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
    break;
  case FPS:
    viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if (setAngle != FPS)
    viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv) {
  std::cout << "starting enviroment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY; // XY;
  initCamera(setAngle, viewer);
  //  simpleHighway(viewer);

  std::shared_ptr<ProcessPointClouds<pcl::PointXYZI>> point_processor =
      std::make_shared<ProcessPointClouds<pcl::PointXYZI>>();
  auto stream = point_processor->streamPcd(
      "/home/saminda/Projects/SFND_Lidar_Obstacle_Detection/src/sensors/data/"
      "pcd/data_1");
  auto stream_iter = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;

  while (!viewer->wasStopped()) {
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    input_cloud = point_processor->loadPcd((*stream_iter).string());
    cityBlock(viewer, point_processor, input_cloud);
    ++stream_iter;
    if (stream_iter == stream.end()) {
      stream_iter = stream.begin();
    }

    viewer->spinOnce();
  }
}
