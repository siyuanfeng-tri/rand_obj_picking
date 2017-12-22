#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "robot_bridge/iiwa_controller.h"
#include "perception/perception.h"
#include "perception/point_cloud_fusion.h"
#include "robot_bridge/iiwa_controller.h"
#include "rgbd_bridge/real_sense_sr300.h"

#include "util.h"

int main(int argc, char** argv) {
  lcm::LCM lcm;

  // Load model.
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr model =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
  pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(argv[1], *model);

  // Load scene.
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scene =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
  pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(argv[2], *scene);

  perception::VisualizePointCloudDrake(*scene, &lcm,
      Eigen::Isometry3d::Identity(),
      "DRAKE_POINTCLOUD_scene");

  scene = perception::SubtractTable<pcl::PointXYZRGBNormal>(scene, 0.01);
  scene =
      perception::CutWithWorkSpaceConstraints<pcl::PointXYZRGBNormal>(
        scene,
        Eigen::Vector3f(0.4, -0.3, -0.01),
        Eigen::Vector3f(0.8, 0., 0.2));

  // Initial guess.
  Eigen::Isometry3f tf =
      Eigen::Translation3f(Eigen::Vector3f(std::atof(argv[3]), std::atof(argv[4]), 0)) *
      Eigen::AngleAxisf(std::atof(argv[5]), Eigen::Vector3f::UnitZ());

  // Align.
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr fitted =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();

  double score;
  FitObj(
      perception::DownSample<pcl::PointXYZRGBNormal>(model, 0.004),
      perception::DownSample<pcl::PointXYZRGBNormal>(scene, 0.004),
      fitted, &tf, &score, &lcm);

  std::cout << tf.matrix() << "\n";
}
