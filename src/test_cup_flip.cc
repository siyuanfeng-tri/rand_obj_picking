#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "robot_bridge/iiwa_controller.h"
#include "perception/perception.h"
#include "perception/point_cloud_fusion.h"
#include "robot_bridge/iiwa_controller.h"
#include "rgbd_bridge/real_sense_sr300.h"

#include "util.h"

struct WorkSpace {
  Eigen::Isometry3d center{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d upper_bound;
  Eigen::Vector3d lower_bound;
};

// Cup frame: handle is +x, opening is z. origin at the cup bottom, center.
void FlipCup(
    const Eigen::Isometry3d& cup_pose,
    const std::vector<Eigen::Isometry3d>& grasps_in_cup_frame,
    const Eigen::Vector2d& cup_goal_xy,
    robot_bridge::RobotBridge& robot_comm) {
  const double kLiftZ = 0.1;
  const double kLinMotionDt = 1;
  const double kRotMotionDt = 1.5;

  Eigen::Isometry3d X0;
  // Call real ik to get in the ball park.
  for (size_t i = 0; i < grasps_in_cup_frame.size(); i++) {
    X0 = cup_pose * grasps_in_cup_frame[i];
    std::vector<Eigen::Isometry3d> poses = {Eigen::Translation3d(Eigen::Vector3d(0, 0, 2 * kLiftZ)) * X0, X0};
    std::vector<double> times = {0, 2 * kLinMotionDt};
    std::vector<Eigen::VectorXd> q_rad;

    Eigen::VectorXd q_guess(7);
    q_guess << 0, 0, 0, -90, 0, 90, 0;
    q_guess = q_guess / 180 * M_PI;

    bool ret = robot_bridge::InverseKinTraj(robot_comm.get_robot(), robot_comm.get_tool_frame(), poses, times, q_guess, &q_rad);
    if (ret != true) {
      if (i == grasps_in_cup_frame.size() - 1) {
        std::cout << "alsdkfjadksjf\n";
        exit(-1);
      }
    } else {
      Eigen::VectorXd q0 = q_rad.front();
      Eigen::VectorXd q1 = q0;
      q0[1] -= 0.6;
      robot_comm.MoveJointRadians(
          {q0, q1}, {1.5, 1.5}, true);
      break;
    }
  }

  // go down
  robot_comm.MoveTool(X0, kLinMotionDt, Eigen::Vector6d::Constant(50), true);

  // grab
  robot_comm.CloseGripper();

  // lift
  X0 = Eigen::Translation3d(Eigen::Vector3d(0, 0, kLiftZ)) * X0;
  X0.translation().head<2>() = cup_goal_xy;
  X0.linear() = Eigen::AngleAxis<double>(-0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix() * X0.linear();
  robot_comm.MoveTool(X0, kLinMotionDt, Eigen::Vector6d::Constant(50), true);

  // rotate 1
  X0 = X0 * Eigen::AngleAxis<double>(M_PI / 2., Eigen::Vector3d::UnitY());
  robot_comm.MoveTool(X0, 2 * kRotMotionDt, Eigen::Vector6d::Constant(50), true);
  usleep(3e5);

  // put down.
  X0 = Eigen::Translation3d(Eigen::Vector3d(0, 0, -kLiftZ)) * X0;
  Eigen::Vector6d F_thresh = Eigen::Vector6d::Constant(50);
  F_thresh[5] = 20;
  robot_comm.MoveTool(X0, kLinMotionDt, F_thresh, true);

  // drop
  robot_comm.OpenGripper();

  // rewind
  X0 = X0 * Eigen::AngleAxis<double>(-M_PI / 2., Eigen::Vector3d::UnitY());
  robot_comm.MoveTool(X0, 2 * kRotMotionDt, Eigen::Vector6d::Constant(50), true);
  usleep(3e5);

  // grab
  robot_comm.CloseGripper();

  // lift
  X0 = Eigen::Translation3d(Eigen::Vector3d(0, 0, kLiftZ)) * X0;
  robot_comm.MoveTool(X0, kLinMotionDt, Eigen::Vector6d::Constant(50), true);

  // rotate 1
  X0 = X0 * Eigen::AngleAxis<double>(M_PI / 2., Eigen::Vector3d::UnitY());
  robot_comm.MoveTool(X0, kRotMotionDt, Eigen::Vector6d::Constant(50), true);
  usleep(3e5);

  // put down.
  X0 = Eigen::Translation3d(Eigen::Vector3d(0, 0, -kLiftZ)) * X0;
  robot_comm.MoveTool(X0, kLinMotionDt, F_thresh, true);

  // drop
  robot_comm.OpenGripper();

  // rotate 2
  X0 = X0 * Eigen::AngleAxis<double>(-M_PI / 4., Eigen::Vector3d::UnitY());
  robot_comm.MoveTool(X0, kRotMotionDt, Eigen::Vector6d::Constant(50), true);
  usleep(3e5);

  // grab
  robot_comm.CloseGripper();

  // lift
  X0 = Eigen::Translation3d(Eigen::Vector3d(0, 0, 2 * kLiftZ)) * X0;
  robot_comm.MoveTool(X0, kLinMotionDt, Eigen::Vector6d::Constant(50), true);
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Scan(
    const rgbd_bridge::RealSenseSR300& camera_interface,
    const std::vector<Eigen::VectorXd>& qs_deg,
    const rgbd_bridge::ImageType depth_type,
    lcm::LCM* lcm,
    robot_bridge::RobotBridge& robot_comm) {
  std::function<Eigen::Vector2f(const Eigen::Vector3f &)> proj_func =
      std::bind(&rgbd_bridge::RealSenseSR300::Project, &camera_interface, depth_type, std::placeholders::_1);
  perception::PointCloudFusion fusion(proj_func, 0.002);

  robot_comm.MoveJointDegrees(qs_deg.front(), 2, true);
  std::vector<Eigen::VectorXd> qq(qs_deg.begin() + 1, qs_deg.end());
  robot_comm.MoveJointDegrees(qq, std::vector<double>(qq.size(), 1.5), false);

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

    Eigen::Isometry3f world_obj_pose;
    fusion.GetLatestFusedPointCloudAndWorldFrameUpdate(&fused_cloud,
                                                        &world_obj_pose);
    perception::VisualizePointCloudDrake(*fused_cloud, lcm,
        Eigen::Isometry3d::Identity(), "DRAKE_POINTCLOUD_FUSED");

    auto status = robot_comm.GetRobotMotionStatus();
    if (status != robot_bridge::MotionStatus::EXECUTING)
      break;
  }

  return fused_cloud->makeShared();
}

int main(int argc, char **argv) {
  // Define workspace.
  WorkSpace input_tray;
  input_tray.center.translation() = Eigen::Vector3d(0.57, -0.5, 0);
  input_tray.upper_bound = Eigen::Vector3d(0.15, 0.21, 0.3);
  input_tray.lower_bound = Eigen::Vector3d(-0.15, -0.21, -0.01);

  WorkSpace staging;
  staging.center.translation() = Eigen::Vector3d(0.5, 0.2, 0);
  staging.upper_bound = Eigen::Vector3d(0.2, 0.2, 0.3);
  staging.lower_bound = Eigen::Vector3d(-0.2, -0.2, -0.01);

  lcm::LCM lcm;
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      IIWA_MODEL_PATH, drake::multibody::joints::kFixed, nullptr, &tree);

  Eigen::Isometry3d X_7C = Eigen::Isometry3d::Identity();
  X_7C.matrix() <<
        0.3826,    -0.880474,      0.27997,   -0.0491369,
        0.923914,     0.364806,    -0.115324,   0.00836689,
        -0.000594751,     0.302791,     0.953057,     0.135499,
                   0,            0,            0,            1;
  RigidBodyFrame<double> camera_frame(
      "Camera", tree.FindBody(robot_bridge::kEEName), X_7C);

  const Eigen::Isometry3d X_7T =
      Eigen::Translation3d(Eigen::Vector3d(0, 0, 0.23)) *
      Eigen::AngleAxis<double>(-22. / 180. * M_PI, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d::UnitY());
  RigidBodyFrame<double> tool_frame("Tool",
                                    tree.FindBody(robot_bridge::kEEName), X_7T);

  // start robot bridge.
  robot_bridge::IiwaController robot_comm(tree, tool_frame, camera_frame);
  robot_comm.Start();

  // start rgbd bridge.
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

  // Load model.
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr model =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
  pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>("cup0_up.pcd", *model);
  model = perception::DownSample<pcl::PointXYZRGBNormal>(model, 0.004);

  // Load scan setpoints.
  //std::vector<Eigen::VectorXd> scan_q_deg = read_q(argv[1], tree.get_num_positions());
  std::vector<Eigen::VectorXd> scan_q_deg =
      read_q("staging_q_deg", tree.get_num_positions());

  // robot_bridge::RobotState robot_state(&tree, &tool_frame);
  // Eigen::Isometry3d X0 = Eigen::Isometry3d::Identity();

  // Reset.
  robot_comm.OpenGripper();
  Eigen::VectorXd q0(7);
  // q0 << 41.1795,  59.8544, -72.0594,  -94.348,  15.4208,  47.2948,   92.208;
  q0 << 0, 0, 0, -90, 0, 90, 0;



  for (int cup_ctr = 0; cup_ctr< 3; cup_ctr++) {
    robot_comm.MoveJointDegrees(q0, 2, true);

    // scan scene.
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scene =
      Scan(camera_interface, scan_q_deg, depth_type, &lcm, robot_comm);

    pcl::io::savePCDFileASCII("scene.pcd", *scene);

    // fit cup.
    std::cout << "press enter to fit model.\n";

    scene = perception::SubtractTable<pcl::PointXYZRGBNormal>(scene, 0.01);
    scene =
      perception::CutWithWorkSpaceConstraints<pcl::PointXYZRGBNormal>(
          scene,
          staging.lower_bound.cast<float>(),
          staging.upper_bound.cast<float>(),
          staging.center.cast<float>());
    perception::VisualizePointCloudDrake(*scene, &lcm,
        Eigen::Isometry3d::Identity(), "DRAKE_POINTCLOUD_cropped_scene");

    Eigen::Isometry3f cup_pose =
      Eigen::Translation3f(staging.center.translation().cast<float>()) *
      Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr fitted =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();

    FitObj(
        perception::DownSample<pcl::PointXYZRGBNormal>(model, 0.004),
        perception::DownSample<pcl::PointXYZRGBNormal>(scene, 0.004),
        fitted, &cup_pose, &lcm);

    // try flip.
    std::cout << "press enter to flip cup.\n";
    // getchar();
    std::vector<Eigen::Isometry3d> grasps_in_cup_frame;

    grasps_in_cup_frame.push_back(
        Eigen::Translation3d(Eigen::Vector3d(0, 0, 0.04)) *
        Eigen::AngleAxis<double>(-1, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxis<double>(-M_PI / 4., Eigen::Vector3d::UnitY()));

    // Sanity check.
    if (cup_pose.linear()(2, 2) < 0.95) {
      std::cout << "Cup should be vertical!\n";
      exit(-1);
    }
    float ang = std::atan2(cup_pose.linear()(1, 0),
        cup_pose.linear()(0, 0));
    cup_pose.linear() = Eigen::AngleAxisf(ang, Eigen::Vector3f::UnitZ()).toRotationMatrix();

    Eigen::Isometry3d cup_in_world = cup_pose.cast<double>();
    // Eigen::Translation3d(Eigen::Vector3d(0.55, -0.25, 0.0)) *
    // Eigen::AngleAxis<double>(-M_PI / 2., Eigen::Vector3d::UnitZ());

    FlipCup(cup_in_world, grasps_in_cup_frame,
        staging.center.translation().head<2>(), robot_comm);

    // move to dish tray.
    Eigen::VectorXd dish_tray_deg0(7);
    dish_tray_deg0 << -77, 17, -19, -89, 6, 76, 20;
    robot_comm.MoveJointDegrees(dish_tray_deg0, 2, true);

    Eigen::Isometry3d X_dish_tray0 =
      Eigen::Translation3d(Eigen::Vector3d(-0.18, -0.5 - 0.1 * cup_ctr, 0.35)) *
      Eigen::AngleAxisd(-M_PI / 2., Eigen::Vector3d::UnitZ());
    robot_comm.MoveTool(X_dish_tray0, 2, true);

    robot_comm.MoveStraightUntilTouch(
        Eigen::Vector3d::UnitZ(), -0.1,
        Eigen::Vector3d(100, 100, 25), true);

    robot_comm.OpenGripper();

    robot_comm.MoveTool(X_dish_tray0, 2, true);
  }


  while(true) {
    ; //std::cout << robot_comm.GetJointPositionDegrees().transpose() << "\n";
  }

  return 0;
}


