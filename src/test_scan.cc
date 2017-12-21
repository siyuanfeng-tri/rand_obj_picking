#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "perception/perception.h"
#include "perception/point_cloud_fusion.h"
#include "robot_bridge/iiwa_controller.h"
#include "rgbd_bridge/real_sense_sr300.h"

#include "util.h"

#include <iostream>
#include <fstream>

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cout << "need to input filename and output.\n";
    exit(-1);
  }

  std::vector<Eigen::VectorXd> qs = read_q(std::string(argv[1]), 7);
  for (const auto& q : qs) {
    std::cout << q.transpose() << "\n";
  }

  lcm::LCM lcm;
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      IIWA_MODEL_PATH, drake::multibody::joints::kFixed, nullptr, &tree);

  // Setup camera and gripper frames.
  Eigen::Isometry3d X_7C = Eigen::Isometry3d::Identity();
  X_7C.matrix() <<
        0.3826,    -0.880474,      0.27997,   -0.0491369,
        0.923914,     0.364806,    -0.115324,   0.00836689,
        -0.000594751,     0.302791,     0.953057,     0.135499,
                   0,            0,            0,            1;

  rgbd_bridge::RealSenseSR300 camera_interface;

  const rgbd_bridge::ImageType depth_type =
      rgbd_bridge::ImageType::RECT_RGB_ALIGNED_DEPTH;
  const rgbd_bridge::ImageType color_type =
      rgbd_bridge::ImageType::RECT_RGB;
  const std::vector<rgbd_bridge::ImageType> channels = {color_type,
                                                        depth_type};
  camera_interface.Start(
      {color_type, depth_type, rgbd_bridge::ImageType::DEPTH}, color_type);
  camera_interface.set_mode(rs_ivcam_preset::RS_IVCAM_PRESET_SHORT_RANGE);

  RigidBodyFrame<double> camera_frame(
      "Camera", tree.FindBody(robot_bridge::kEEName), X_7C);

  const Eigen::Isometry3d X_7T =
      Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.24)) *
      Eigen::AngleAxis<double>(-22. / 180. * M_PI, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d::UnitY());
  RigidBodyFrame<double> tool_frame("Tool",
                                    tree.FindBody(robot_bridge::kEEName), X_7T);

  robot_bridge::IiwaController robot_comm(tree, tool_frame, camera_frame);
  robot_comm.Start();

  std::function<Eigen::Vector2f(const Eigen::Vector3f &)> proj_func =
      std::bind(&rgbd_bridge::RealSenseSR300::Project, &camera_interface, depth_type, std::placeholders::_1);
  perception::PointCloudFusion fusion(proj_func, 0.002);

  robot_comm.MoveJointDegrees(qs.front(), 2, true);

  robot_comm.MoveJointDegrees(qs, std::vector<double>(qs.size(), 2), false);

  Eigen::Isometry3f camera_pose;
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr raw_cloud;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr fused_cloud;
  boost::shared_ptr<const cv::Mat> raw_depth;
  uint64_t timestamp;

  while (true) {
    raw_cloud = camera_interface.GetLatestPointCloud(&timestamp);
    raw_depth = camera_interface.GetLatestImage(depth_type, &timestamp);
    if (!raw_cloud || !raw_depth)
      continue;

    camera_pose = robot_comm.GetCameraPose().cast<float>();

    bool err = fusion.ProcFrame(camera_pose, *raw_cloud, *raw_depth);
    if (!err)
      continue;

    Eigen::Isometry3f world_update;
    fusion.GetLatestFusedPointCloudAndWorldFrameUpdate(&fused_cloud,
                                                        &world_update);
    perception::VisualizePointCloudDrake(*fused_cloud, &lcm,
        Eigen::Isometry3d::Identity(), "DRAKE_POINTCLOUD_FUSED");

    auto status = robot_comm.GetRobotMotionStatus();
    if (status != robot_bridge::MotionStatus::EXECUTING)
      break;
  }

  robot_comm.Stop();

  /*
  auto cloud =
    perception::CutWithWorkSpaceConstraints<pcl::PointXYZRGBNormal>(
        fused_cloud,
        Eigen::Vector3f(0.48, -0.1, -0.01),
        Eigen::Vector3f(0.68, 0.1, 0.2));
  cloud = perception::SubtractTable<pcl::PointXYZRGBNormal>(cloud, 0.005);
  */
  auto cloud = perception::SOROutlierRemoval<pcl::PointXYZRGBNormal>(fused_cloud);

  pcl::io::savePCDFileASCII(std::string(argv[2]), *cloud);

  return 0;
}
