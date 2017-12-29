#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "perception/perception.h"
#include "perception/point_cloud_fusion.h"
#include "rgbd_bridge/real_sense_sr300.h"
#include "robot_bridge/iiwa_controller.h"
#include "robot_bridge/iiwa_controller.h"

#include "util.h"

struct WorkSpace {
  Eigen::Isometry3d center{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d upper_bound;
  Eigen::Vector3d lower_bound;
};

/**
 * Moves a cup from the input tray to the staging area.
 * For each grasp in @p grasps_in_cup_frame, uses iktraj to generate a 4 knot
 * trajectory (pose, pick, pose, place). If ik successes, the robot will move
 * to the first q for such a traj, then exec a linear motion down to attempt
 * the grasp. If the attempt results in collision or empty grasp, the robot
 * slowly backs away to the posture this behavior starts with and return false.
 *
 * If success, the cup should be placed in the staging area, with the handle
 * facing forward.
 * If fail, the robot should be back to where this behavior starts, with the
 * gripper open.
 */
bool MoveFromTrayToStage(
    const Eigen::Isometry3d &cup_pose,
    const std::vector<Eigen::Isometry3d> &grasps_in_cup_frame,
    const Eigen::Isometry3d &cup_goal, robot_bridge::RobotBridge &robot_comm) {
  const double kLiftZ = 0.1;

  Eigen::VectorXd q_now = robot_comm.GetJointPositionRadians();

  Eigen::Isometry3d X0;
  // Call real ik to get in the ball park.
  for (size_t i = 0; i < grasps_in_cup_frame.size(); i++) {
    X0 = cup_pose * grasps_in_cup_frame[i];
    Eigen::Isometry3d X1 = cup_goal * grasps_in_cup_frame[i];
    std::vector<Eigen::Isometry3d> poses = {
        Eigen::Translation3d(Eigen::Vector3d(0, 0, 2 * kLiftZ)) * X0, X0,
        Eigen::Translation3d(Eigen::Vector3d(0, 0, 2 * kLiftZ)) * X1, X1,
    };
    std::vector<double> times = {0, 1, 2, 3};
    std::vector<Eigen::VectorXd> q_rad;

    Eigen::VectorXd q_guess(7);
    q_guess << 0, 0, 0, -90, 0, 90, 0;
    q_guess = q_guess / 180 * M_PI;

    bool ret = robot_bridge::InverseKinTraj(robot_comm.get_robot(),
                                            robot_comm.get_tool_frame(), poses,
                                            times, q_guess, &q_rad);

    if (ret != true) {
      if (i == grasps_in_cup_frame.size() - 1) {
        std::cout << "Can't plan a motion to move from tray to staging area.\n";
        return false;
      }
    } else {
      // Pose for grasp.
      robot_comm.MoveJointRadians(q_rad[0], true);

      // Approach
      Eigen::Vector6d F_u = Eigen::Vector6d::Constant(50);
      Eigen::Vector6d F_l = Eigen::Vector6d::Constant(-50);
      F_u[5] = 25;
      F_l[5] = -25;

      auto status = robot_comm.MoveTool(X0, 1, F_u, F_l, true);
      // If we do hit stuff, go back slowly, and abort.
      if (status == robot_bridge::MotionStatus::ERR_FORCE_SAFETY) {
        robot_comm.MoveJointRadians({q_rad[0], q_now}, {2, 1}, true);
        return false;
      }

      if (!robot_comm.CloseGripper()) {
        // Didn't grasp stuff, should abort.
        robot_comm.MoveJointRadians({q_rad[0], q_now}, {2, 1}, true);
        robot_comm.OpenGripper();
        return false;
      }

      // Retract
      robot_comm.MoveJointRadians(q_rad[0], true);

      // Pose for place.
      robot_comm.MoveJointRadians(q_rad[2], true);

      // Place.
      robot_comm.MoveStraightUntilTouch(Eigen::Vector3d::UnitZ(), -0.2,
                                        Eigen::Vector3d(100, 100, 20),
                                        Eigen::Vector3d::Constant(-100), true);

      robot_comm.OpenGripper();

      // Retract.
      robot_comm.MoveTool(
          Eigen::Translation3d(Eigen::Vector3d(0, 0, 1.5 * kLiftZ)) * X1, 1,
          true);

      break;
    }
  }

  return true;
}

/**
 * Moves the cup from staging area to the dish rack.
 *
 * Assumes that the cup is already being grasped in the correct orientation
 * (from FlipCup).
 *
 * First exec a Move J to the general vicinity above the rack, then exec a
 * linear motion to @p cup_pose_above_rack, then move straight down until
 * Fz is > 25N, then release and retract. During retractiont, if a big external
 * force is sensed, throws exception (no automatic recovery should be attempted
 * in this case since the gripper is likely to be caught on the rack.)
 */
void MoveFromStageToRack(const Eigen::Isometry3d &cup_pose_above_rack,
                         robot_bridge::RobotBridge &robot_comm) {
  Eigen::VectorXd q_above_rack(7);
  q_above_rack << -77, 17, -19, -89, 6, 76, 20;

  // MoveJ to right above rack.
  robot_comm.MoveJointDegrees(q_above_rack, true);

  // MoveL to right above the placement.
  robot_comm.MoveTool(cup_pose_above_rack, 1, true);

  // Place until Fz > 25.
  robot_comm.MoveStraightUntilTouch(Eigen::Vector3d::UnitZ(), -0.2,
                                    Eigen::Vector3d(100, 100, 25),
                                    Eigen::Vector3d::Constant(-100), true);

  // Release.
  robot_comm.OpenGripper();

  // Retract with force guard, incase the we are pulling the rack with us.
  Eigen::Vector6d F_u = Eigen::Vector6d::Constant(30);
  Eigen::Vector6d F_l = Eigen::Vector6d::Constant(-30);
  auto status = robot_comm.MoveTool(cup_pose_above_rack, 1, F_u, F_l, true);

  // Don't attempt to recover.
  if (status == robot_bridge::MotionStatus::ERR_FORCE_SAFETY) {
    throw std::runtime_error("Catching rack when retracting.");
  }
}

/**
 * Flips a cup upside down with 2 90 deg motions. First motion puts the cup
 * flat on the desk, second motion flips it upside down.
 * Throws exceptions if any motion primitive returns with errors. No automatic
 * recovery is attempted here.
 *
 * This behavior performs a scan for the cup, so there is no strict assumption
 * of the cup's pose, however, motion plan could fail, also linear motions
 * could get in singularity / joint limit, which would both result in this
 * behavior throw exceptions.
 */
void FlipCup(const Eigen::Isometry3d &cup_pose,
             const std::vector<Eigen::Isometry3d> &grasps_in_cup_frame,
             const Eigen::Vector2d &cup_goal_xy,
             robot_bridge::RobotBridge &robot_comm) {
  const double kLiftZ = 0.1;
  const double kLinMotionDt = 0.5;

  Eigen::Isometry3d X0;
  // Call real ik to get in the ball park.
  for (size_t i = 0; i < grasps_in_cup_frame.size(); i++) {
    X0 = cup_pose * grasps_in_cup_frame[i];
    std::vector<Eigen::Isometry3d> poses = {
        Eigen::Translation3d(Eigen::Vector3d(0, 0, 2 * kLiftZ)) * X0, X0};
    std::vector<double> times = {0, 2 * kLinMotionDt};
    std::vector<Eigen::VectorXd> q_rad;

    Eigen::VectorXd q_guess(7);
    q_guess << 0, 0, 0, -90, 0, 90, 0;
    q_guess = q_guess / 180 * M_PI;

    bool ret = robot_bridge::InverseKinTraj(robot_comm.get_robot(),
                                            robot_comm.get_tool_frame(), poses,
                                            times, q_guess, &q_rad);
    if (ret != true) {
      if (i == grasps_in_cup_frame.size() - 1) {
        throw std::runtime_error("Ik doesn't know how to flip this mug :(");
      }
    } else {
      robot_comm.MoveJointRadians(q_rad.front(), true);
      break;
    }
  }

  // go down
  Eigen::Vector6d F_u = Eigen::Vector6d::Constant(25);
  Eigen::Vector6d F_l = Eigen::Vector6d::Constant(-25);
  if (robot_comm.MoveTool(X0, 1, F_u, F_l, true) != robot_bridge::MotionStatus::DONE) {
    throw std::runtime_error("Motion error.");
  }

  // grab
  robot_comm.CloseGripper();
  robot_bridge::GripperState wsg_state;
  robot_comm.GetGripperState(&wsg_state);
  if (wsg_state.get_position() <= 0.05) {
    throw std::runtime_error("Empty grasp flipping the cup.");
  }

  // lift
  X0 = Eigen::Translation3d(Eigen::Vector3d(0, 0, kLiftZ)) * X0;
  X0.translation().head<2>() = cup_goal_xy;
  X0.linear() = Eigen::AngleAxis<double>(-0.3, Eigen::Vector3d::UnitZ())
                    .toRotationMatrix() *
                X0.linear();
  if (robot_comm.MoveTool(X0, kLinMotionDt, true) != robot_bridge::MotionStatus::DONE) {
    throw std::runtime_error("Motion error.");
  }

  // rotate 1
  X0 = X0 * Eigen::AngleAxis<double>(M_PI / 2., Eigen::Vector3d::UnitY());
  if (robot_comm.MoveTool(X0, 2, true) != robot_bridge::MotionStatus::DONE) {
    throw std::runtime_error("Motion error.");
  }

  // put down.
  X0 = Eigen::Translation3d(Eigen::Vector3d(0, 0, -kLiftZ)) * X0;
  if (robot_comm.MoveTool(X0, kLinMotionDt, true) != robot_bridge::MotionStatus::DONE) {
    throw std::runtime_error("Motion error.");
  }

  // drop
  robot_comm.OpenGripper();

  // rewind
  X0 = X0 * Eigen::AngleAxis<double>(-M_PI / 2., Eigen::Vector3d::UnitY());
  X0 = Eigen::Translation3d(Eigen::Vector3d(0, 0, -0.015)) * X0;
  if (robot_comm.MoveTool(X0, 2., true) != robot_bridge::MotionStatus::DONE) {
    throw std::runtime_error("Motion error.");
  }

  // grab
  robot_comm.CloseGripper();
  robot_comm.GetGripperState(&wsg_state);
  if (wsg_state.get_position() <= 0.05) {
    throw std::runtime_error("Empty grasp flipping the cup.");
  }

  // lift
  X0 = Eigen::Translation3d(Eigen::Vector3d(0, 0, kLiftZ)) * X0;
  if (robot_comm.MoveTool(X0, kLinMotionDt, true) != robot_bridge::MotionStatus::DONE) {
    throw std::runtime_error("Motion error.");
  }

  // rotate 2
  X0 = X0 * Eigen::AngleAxis<double>(M_PI / 2., Eigen::Vector3d::UnitY());
  if (robot_comm.MoveTool(X0, 2, true) != robot_bridge::MotionStatus::DONE) {
    throw std::runtime_error("Motion error.");
  }

  // put down.
  X0 = Eigen::Translation3d(Eigen::Vector3d(0, 0, -kLiftZ)) * X0;
  if (robot_comm.MoveTool(X0, kLinMotionDt, true) != robot_bridge::MotionStatus::DONE) {
    throw std::runtime_error("Motion error.");
  }

  // drop
  robot_comm.OpenGripper();

  // rotate 2
  X0 = X0 * Eigen::AngleAxis<double>(-M_PI / 4., Eigen::Vector3d::UnitY());
  // rotate a bit more to avoid the finger pad getting caught on the handle when
  // releasing. in the dish rack.
  X0 = X0 * Eigen::AngleAxis<double>(-0.35, Eigen::Vector3d::UnitZ());
  // X0 = X0 * Eigen::Translation3d(Eigen::Vector3d(-0.005, 0, 0));
  if (robot_comm.MoveTool(X0, 1.5, true) != robot_bridge::MotionStatus::DONE) {
    throw std::runtime_error("Motion error.");
  }

  // grab
  robot_comm.CloseGripper();
  robot_comm.GetGripperState(&wsg_state);
  if (wsg_state.get_position() <= 0.05) {
    throw std::runtime_error("Empty grasp flipping the cup.");
  }

  // lift
  X0 = Eigen::Translation3d(Eigen::Vector3d(0, 0, 2 * kLiftZ)) * X0;
  if (robot_comm.MoveTool(X0, kLinMotionDt, true) != robot_bridge::MotionStatus::DONE) {
    throw std::runtime_error("Motion error.");
  }
}

// Scans an area and returns the merged point cloud.
// set points are defined in @p &qs_deg, which is assumed to be in degrees!
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr
Scan(const rgbd_bridge::RealSenseSR300 &camera_interface,
     const std::vector<Eigen::VectorXd> &qs_deg,
     const rgbd_bridge::ImageType depth_type, lcm::LCM *lcm,
     robot_bridge::RobotBridge &robot_comm) {
  std::function<Eigen::Vector2f(const Eigen::Vector3f &)> proj_func =
      std::bind(&rgbd_bridge::RealSenseSR300::Project, &camera_interface,
                depth_type, std::placeholders::_1);
  perception::PointCloudFusion fusion(proj_func, 0.002);

  robot_comm.MoveJointDegrees(qs_deg.front(), true);

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
                                         Eigen::Isometry3d::Identity(),
                                         "DRAKE_POINTCLOUD_FUSED");

    auto status = robot_comm.GetRobotMotionStatus();
    if (status != robot_bridge::MotionStatus::EXECUTING)
      break;
  }

  return fused_cloud->makeShared();
}

std::vector<Eigen::Isometry3d> GeneratePlacementGoalsAboveRack() {
  std::vector<Eigen::Isometry3d> pose_above_rack;
  // row 1
  for (int i = 0; i < 3; i++) {
    pose_above_rack.push_back(
        Eigen::Translation3d(Eigen::Vector3d(-0.182, -0.52 - 0.12 * i, 0.27)) *
        Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitZ()));
  }
  // row 2
  for (int i = 0; i < 3; i++) {
    pose_above_rack.push_back(
        Eigen::Translation3d(Eigen::Vector3d(-0.099, -0.47 - 0.12 * i, 0.27)) *
        Eigen::AngleAxisd(-M_PI / 2., Eigen::Vector3d::UnitZ()));
  }
  // row 3
  for (int i = 0; i < 3; i++) {
    pose_above_rack.push_back(
        Eigen::Translation3d(Eigen::Vector3d(0.035, -0.5 - 0.12 * i, 0.27)) *
        Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitZ()));
  }
  // row 4
  for (int i = 0; i < 3; i++) {
    pose_above_rack.push_back(
        Eigen::Translation3d(Eigen::Vector3d(0.196, -0.5 - 0.12 * i, 0.27)) *
        Eigen::AngleAxisd(-M_PI / 2., Eigen::Vector3d::UnitZ()));
  }
  return pose_above_rack;
}

Eigen::Isometry3f RectifyCupPose(const Eigen::Isometry3f &cup_pose) {
  if (cup_pose.linear()(2, 2) < 0.95) {
    throw std::runtime_error("Cup should be vertical.\n");
  }

  // Hack:
  // I am getting rid of the roll and pitch angle here...
  Eigen::Isometry3f ret = cup_pose;
  float ang = std::atan2(cup_pose.linear()(1, 0), cup_pose.linear()(0, 0));
  ret.linear() =
      Eigen::AngleAxisf(ang, Eigen::Vector3f::UnitZ()).toRotationMatrix();

  return ret;
}

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cout << "Need to input # of cups in the dish rack.\n";
    exit(-1);
  }
  const int num_cups_in_rack = std::atoi(argv[1]);

  // Define workspace.
  WorkSpace input_tray;
  input_tray.center.translation() = Eigen::Vector3d(0.53, -0.38, 0);
  input_tray.upper_bound = Eigen::Vector3d(0.14, 0.20, 0.13);
  input_tray.lower_bound = Eigen::Vector3d(-0.14, -0.2, -0.01);

  WorkSpace staging;
  staging.center.translation() = Eigen::Vector3d(0.5, 0.2, 0);
  staging.upper_bound = Eigen::Vector3d(0.2, 0.2, 0.13);
  staging.lower_bound = Eigen::Vector3d(-0.2, -0.2, -0.01);

  // Declare a bunch of stuff
  lcm::LCM lcm;
  RigidBodyTree<double> tree;
  const std::string robot_model_path = std::string(MODEL_DIR) +
      "iiwa_description/urdf/iiwa14_polytope_collision.urdf";
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      robot_model_path, drake::multibody::joints::kFixed, nullptr, &tree);

  // Camera "C" frame relative to link7's frame.
  Eigen::Isometry3d X_7C = Eigen::Isometry3d::Identity();
  X_7C.matrix() << 0.3826, -0.880474, 0.27997, -0.0491369, 0.923914, 0.364806,
      -0.115324, 0.00836689, -0.000594751, 0.302791, 0.953057, 0.135499, 0, 0,
      0, 1;
  RigidBodyFrame<double> camera_frame(
      "Camera", tree.FindBody("iiwa_link_7"), X_7C);

  // Tool "T" frame wrt link7.
  const Eigen::Isometry3d X_7T =
      Eigen::Translation3d(Eigen::Vector3d(0, 0, 0.23)) *
      Eigen::AngleAxis<double>(-22. / 180. * M_PI, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d::UnitY());
  RigidBodyFrame<double> tool_frame("Tool",
                                    tree.FindBody("iiwa_link_7"), X_7T);

  // Start robot bridge.
  robot_bridge::IiwaController robot_comm(tree, tool_frame, camera_frame);
  robot_comm.Start();

  // Start rgbd bridge.
  rgbd_bridge::RealSenseSR300 camera_interface;
  const rgbd_bridge::ImageType depth_type =
      rgbd_bridge::ImageType::RECT_RGB_ALIGNED_DEPTH;
  const rgbd_bridge::ImageType color_type = rgbd_bridge::ImageType::RECT_RGB;
  const std::vector<rgbd_bridge::ImageType> channels = {color_type, depth_type};
  camera_interface.Start(
      {color_type, depth_type, rgbd_bridge::ImageType::DEPTH}, color_type);
  camera_interface.set_mode(rs_ivcam_preset::RS_IVCAM_PRESET_SHORT_RANGE);

  // Load cup template as a point cloud.
  // Cup frame: handle is +x, opening is z. origin at the cup bottom, center.
  // This cloud is already "centered" (world frame and cup frame coincide.)
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cup_template =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
  pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(
      std::string(MODEL_DIR) + "pcd/cup0_up.pcd", *cup_template);
  cup_template =
      perception::DownSample<pcl::PointXYZRGBNormal>(cup_template, 0.005);

  // Load scan motions' setpoints.
  std::vector<Eigen::VectorXd> stage_q_deg =
      read_q(std::string(CONFIG_DIR) + "q_waypoints/staging_deg",
             tree.get_num_positions());

  std::vector<Eigen::VectorXd> input_q_deg =
      read_q(std::string(CONFIG_DIR) + "q_waypoints/input_tray_deg",
             tree.get_num_positions());

  // "Prep" postures.
  Eigen::VectorXd tray_prep_q_deg(7);
  tray_prep_q_deg << -41, 17, -1, -82, 5, 78, 10;

  Eigen::VectorXd stage_prep_q_deg(7);
  stage_prep_q_deg << 26, 17, -1, -82, 5, 78, 11;

  // Reset robot.
  robot_comm.OpenGripper();
  Eigen::VectorXd q0(7);
  q0 << 0, 0, 0, -90, 0, 90, 0;

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scene;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr fitted =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
  Eigen::Isometry3f cup_pose;
  double score;

  // Generate placement goals in the dish rack.
  std::vector<Eigen::Isometry3d> pose_above_rack =
      GeneratePlacementGoalsAboveRack();

  // Move cups from tray to rack until no cups are detected in the tray.
  for (int cup_ctr = num_cups_in_rack; cup_ctr < (int)pose_above_rack.size();) {
    //////////////////////////////////////////////////////////////////////////
    // Transfer from input tray
    //////////////////////////////////////////////////////////////////////////

    // Scan input tray.
    scene = Scan(camera_interface, input_q_deg, depth_type, &lcm, robot_comm);
    scene = perception::SubtractTable<pcl::PointXYZRGBNormal>(scene, 0.01);
    scene = perception::CutWithWorkSpaceConstraints<pcl::PointXYZRGBNormal>(
        scene, input_tray.lower_bound.cast<float>(),
        input_tray.upper_bound.cast<float>(), input_tray.center.cast<float>());
    // Debug info.
    pcl::io::savePCDFileASCII("tray_scene.pcd", *scene);

    perception::VisualizePointCloudDrake(*scene, &lcm,
                                         Eigen::Isometry3d::Identity(),
                                         "DRAKE_POINTCLOUD_input_tray");

    // Move to prep posture with async move, while segmentation and pose
    // estimation happen in parallel.
    robot_comm.MoveJointDegrees(tray_prep_q_deg, 1.5, false);

    // Segement individual cups.
    auto clusters = perception::SplitIntoClusters<pcl::PointXYZRGBNormal>(scene, 0.005, 5000, 100000);
    std::cout << "NUMBER OF MUGS: " << clusters.size() << "\n";
    if (clusters.size() == 0) {
      std::cout << "NO MORE MUGS.\n";
      break;
    }

    // Estimate the pose for the first mug..
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &first_mug =
        clusters.front();
    Eigen::Vector4f mid_pt;
    pcl::compute3DCentroid(*first_mug, mid_pt);

    std::vector<Eigen::Isometry3f> cup_guess;
    const int num_ang_bin = 12;
    // Making initial guesses with pos = centroild of alined template (offseted
    // by z), and the yaw angle is gridded by num_ang_bin.
    for (int i = 0; i < num_ang_bin; i++) {
      cup_guess.push_back(
          Eigen::Translation3f(mid_pt.head<3>()) *
          Eigen::AngleAxisf((2 * i) * M_PI / (double)num_ang_bin,
                            Eigen::Vector3f::UnitZ()));
    }
    score = ThreadedFitObj(
        cup_template,
        perception::DownSample<pcl::PointXYZRGBNormal>(first_mug, 0.005),
        cup_guess, fitted, &cup_pose, &lcm);

    std::cout << "score: " << score << "\n";
    // Normal good fit should be around 1.x e-5
    if (score > 6e-5) {
      std::cout
          << "\n\n Uncertain about alignement, press Enter to proceed. \n\n";
      getchar();
    }

    // Block until we are done with async move.
    robot_comm.WaitForRobotMotionCompletion();

    // Call ik to move the first mug to staging area.
    {
      // 2 candidate pre-defined grasps (forehand / backhand).
      std::vector<Eigen::Isometry3d> grasps_in_cup_frame;
      grasps_in_cup_frame.push_back(
          Eigen::Translation3d(Eigen::Vector3d(0, 0, 0.06)) *
          Eigen::AngleAxis<double>(-0, Eigen::Vector3d::UnitZ()));
      grasps_in_cup_frame.push_back(
          Eigen::Translation3d(Eigen::Vector3d(0, 0, 0.06)) *
          Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d::UnitZ()));

      cup_pose = RectifyCupPose(cup_pose);

      // Exec script.
      bool success =
          MoveFromTrayToStage(cup_pose.cast<double>(), grasps_in_cup_frame,
                              staging.center, robot_comm);

      if (!success)
        continue;
    }

    //////////////////////////////////////////////////////////////////////////
    // Flip cup in the staging area.
    //////////////////////////////////////////////////////////////////////////

    // Scan staging area, sometimes the release is not clean and the mug moves.
    scene = Scan(camera_interface, stage_q_deg, depth_type, &lcm, robot_comm);

    // Debug info.
    pcl::io::savePCDFileASCII("scene.pcd", *scene);

    // Move to prep posture with async move, while pose estimations happens
    // in parallel.
    robot_comm.MoveJointDegrees(stage_prep_q_deg, 1.5, false);

    // Estimate cup pose.
    scene = perception::SubtractTable<pcl::PointXYZRGBNormal>(scene, 0.01);
    scene = perception::CutWithWorkSpaceConstraints<pcl::PointXYZRGBNormal>(
        scene, staging.lower_bound.cast<float>(),
        staging.upper_bound.cast<float>(), staging.center.cast<float>());
    perception::VisualizePointCloudDrake(*scene, &lcm,
                                         Eigen::Isometry3d::Identity(),
                                         "DRAKE_POINTCLOUD_staging");

    cup_pose =
        Eigen::Translation3f(staging.center.translation().cast<float>()) *
        Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());

    // Guess initial poses similar to the previous part.
    cup_guess.clear();
    for (int i = 0; i < num_ang_bin; i++) {
      cup_guess.push_back(
          Eigen::Translation3f(staging.center.translation().cast<float>()) *
          Eigen::AngleAxisf((2 * i) * M_PI / (double)num_ang_bin,
                            Eigen::Vector3f::UnitZ()));
    }
    score = ThreadedFitObj(
        cup_template,
        perception::DownSample<pcl::PointXYZRGBNormal>(scene, 0.005), cup_guess,
        fitted, &cup_pose, &lcm);

    std::cout << "score: " << score << "\n";
    // Normal good fit should be around 1.x e-5
    if (score > 3e-4) {
      std::cout
          << "\n\n Uncertain about alignement, press Enter to proceed. \n\n";
      getchar();
    }

    cup_pose = RectifyCupPose(cup_pose);

    // make sure we are done with async move.
    robot_comm.WaitForRobotMotionCompletion();

    std::vector<Eigen::Isometry3d> grasps_in_cup_frame;
    grasps_in_cup_frame.push_back(
        Eigen::Translation3d(Eigen::Vector3d(0, 0, 0.04)) *
        Eigen::AngleAxis<double>(-1, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxis<double>(-M_PI / 4., Eigen::Vector3d::UnitY()));

    FlipCup(cup_pose.cast<double>(), grasps_in_cup_frame,
            staging.center.translation().head<2>(), robot_comm);

    //////////////////////////////////////////////////////////////////////////
    // Move to dish rack.
    //////////////////////////////////////////////////////////////////////////
    MoveFromStageToRack(pose_above_rack.at(cup_ctr), robot_comm);

    // Successfully placed a cup.
    cup_ctr++;
  }

  return 0;
}
