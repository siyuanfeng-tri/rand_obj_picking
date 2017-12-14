#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "grasp_gen/anti_podal_grasp.h"
#include "perception/perception.h"
#include "perception/point_cloud_fusion.h"
#include "robot_bridge/iiwa_controller.h"

#include "rgbd_bridge/real_sense_sr300.h"

#include <atomic>
#include <stdexcept>
#include <string>

using rgbd_bridge::RealSenseSR300;

struct WorkCell {
  // Prep
  Eigen::VectorXd q_prep_deg{Eigen::VectorXd::Zero(7)};
  // Scane
  Eigen::VectorXd q_scan_left_deg{Eigen::VectorXd::Zero(7)};
  Eigen::VectorXd q_scan_mid_deg{Eigen::VectorXd::Zero(7)};
  Eigen::VectorXd q_scan_right_deg{Eigen::VectorXd::Zero(7)};

  Eigen::Isometry3d world_rot{Eigen::Isometry3d::Identity()};

  Eigen::Isometry3d X_bin{};
  Eigen::Vector3d bin_half_dim{Eigen::Vector3d(0.13, 0.26, 0.03)};

  // Dest bin
  Eigen::VectorXd q_place_deg{Eigen::VectorXd::Zero(7)};
};

bool CheckReachableConstraint(const Eigen::Isometry3d &pose) {
  const double max_x = 0.8;
  const double min_x = 0.40;
  double pose_x = pose.translation()(0);
  double yaw_angle = atan2(-pose.matrix()(0, 1), pose.matrix()(0, 0));
  if (pose_x > min_x && pose_x < max_x && fabs(yaw_angle) < M_PI * 3.0 / 4.0) {
    //std::cout << "The grasp is reachable" << std::endl;
    return true;
  } else {
    //std::cout << "The grasp is NOT reachable" << std::endl;
    //std::cout << pose.translation().transpose() << ", yaw: " << yaw_angle
    //          << "\n";
    return false;
  }
}

bool CheckBinIntersection(
    const Eigen::Isometry3d& grasp,
    double gripper_width,
    const Eigen::Isometry3d& X_WBin,
    const Eigen::Vector3d& half_dim) {
  // grasp is high don't worry.
  if (grasp.translation()[2] > 0.07)
    return false;

  // Eigen::Vector3d left_finger = grasp * Eigen::Vector3d(0, gripper_width, 0);
  // Eigen::Vector3d right_finger = grasp * Eigen::Vector3d(0, -gripper_width, 0);

  // bottom corners of the bin.
  Eigen::Vector3d p1 = X_WBin * Eigen::Vector3d(-half_dim[0], -half_dim[1], -half_dim[2]);
  Eigen::Vector3d p2 = X_WBin * Eigen::Vector3d(half_dim[0], -half_dim[1], -half_dim[2]);
  Eigen::Vector3d p3 = X_WBin * Eigen::Vector3d(-half_dim[0], half_dim[1], -half_dim[2]);
  Eigen::Vector3d p4 = X_WBin * Eigen::Vector3d(half_dim[0], half_dim[1], -half_dim[2]);

  // top corners
  Eigen::Vector3d p5 = X_WBin * Eigen::Vector3d(-half_dim[0], -half_dim[1], half_dim[2]);
  Eigen::Vector3d p6 = X_WBin * Eigen::Vector3d(half_dim[0], half_dim[1], half_dim[2]);

  Eigen::Hyperplane<double, 3> left_wall = Eigen::Hyperplane<double, 3>::Through(p1, p2, p5);
  Eigen::Hyperplane<double, 3> back_wall = Eigen::Hyperplane<double, 3>::Through(p1, p3, p5);
  Eigen::Hyperplane<double, 3> right_wall = Eigen::Hyperplane<double, 3>::Through(p3, p4, p6);
  Eigen::Hyperplane<double, 3> front_wall = Eigen::Hyperplane<double, 3>::Through(p2, p4, p6);

  Eigen::ParametrizedLine<double, 3> closing_line(grasp.translation(), grasp.linear().col(1));
  double left_dist = (closing_line.intersectionPoint(left_wall) - grasp.translation()).norm();
  double right_dist = (closing_line.intersectionPoint(right_wall) - grasp.translation()).norm();
  double front_dist = (closing_line.intersectionPoint(front_wall) - grasp.translation()).norm();
  double back_dist = (closing_line.intersectionPoint(back_wall) - grasp.translation()).norm();

  /*
  std::cout << "bin: " << X_WBin.matrix() << "\n";
  std::cout << "grasp: " << grasp.matrix() << "\n";
  std::cout << "p1: " << p1.transpose() << "\n";
  std::cout << "p2: " << p2.transpose() << "\n";
  std::cout << "p3: " << p3.transpose() << "\n";
  std::cout << "p4: " << p4.transpose() << "\n";

  std::cout << "l: " << left_finger.transpose() << "\n";
  std::cout << "r: " << right_finger.transpose() << "\n";
  std::cout << closing_line.pointAt(gripper_width).transpose() << "\n";
  std::cout << closing_line.pointAt(-gripper_width).transpose() << "\n";

  std::cout << "\n\nToo close to bin! dist: "
              << left_dist << ", "
              << right_dist << ", "
              << front_dist << ", "
              << back_dist << "\n";
    std::cout << closing_line.intersectionPoint(left_wall).transpose() << "\n";
    std::cout << closing_line.intersectionPoint(right_wall).transpose() << "\n";
    std::cout << closing_line.intersectionPoint(front_wall).transpose() << "\n";
    std::cout << closing_line.intersectionPoint(back_wall).transpose() << "\n";
    getchar();
  */

  if (left_dist <= gripper_width ||
      right_dist <= gripper_width ||
      front_dist <= gripper_width ||
      back_dist <= gripper_width) {
    return true;
  } else {
    return false;
  }
}

const AntiPodalGrasp* PickBestGrasp(
    const WorkCell& cell,
    const std::vector<AntiPodalGrasp>& all_grasps,
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr& grasp_cloud) {
  if (all_grasps.empty())
    return nullptr;

  const AntiPodalGrasp* best = nullptr;
  Eigen::Vector4f mid_pt;

  float best_norm = 0;
  float best_dist = 1e5;
  float best_z = -1e5;

  for (const auto& grasp : all_grasps) {
    if (CheckReachableConstraint(cell.world_rot.inverse() * grasp.hand_pose) &&
        !CheckBinIntersection(grasp.hand_pose, 0.06, cell.X_bin, cell.bin_half_dim)) {
      Eigen::Isometry3f mid_finger = grasp.hand_pose.cast<float>();
      // Enclosed points.
      auto enclosed =
          perception::CutWithWorkSpaceConstraints<pcl::PointXYZRGBNormal>(
              grasp_cloud,
              Eigen::Vector3f(-0.01, -0.045, -0.01),
              Eigen::Vector3f(0.01, 0.045, 0.07),
              mid_finger);

      float norm_score = 0;
      Eigen::Vector3f normal;
      for (size_t i = 0; i < enclosed->size(); i++) {
        normal << enclosed->points[i].normal_x,
                  enclosed->points[i].normal_y,
                  enclosed->points[i].normal_z;
        float dot = normal.dot(grasp.hand_pose.linear().col(1).cast<float>());
        norm_score += dot * dot;
      }

      pcl::compute3DCentroid(*enclosed, mid_pt);
      float dist = (mid_pt.head<3>() - mid_finger.translation()).norm();
      float z = grasp.hand_pose.translation()[2];

      if (norm_score >= best_norm) {
        //if (dist <= best_dist) {
          //if (z >= best_z) {
            best_norm = norm_score;
            best_dist = dist;
            best_z = z;
            best = &grasp;
          //}
        //}
      }
    }
  }
  return best;
}

std::mutex g_cloud_mutex;
pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr g_fused_cloud;
int g_fused_cloud_ctr{0};
std::atomic<bool> g_skip_fusion{false};
void CloudThread(robot_bridge::RobotBridge *robot_comm,
                 const rgbd_bridge::RealSenseSR300 *camera_interface,
                 const rgbd_bridge::ImageType depth_type,
                 perception::PointCloudFusion *fusion, lcm::LCM *lcm) {
  Eigen::Isometry3f camera_pose;
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr raw_cloud;
  boost::shared_ptr<const cv::Mat> raw_depth;
  uint64_t timestamp;

  while (true) {
    if (g_skip_fusion)
      continue;

    raw_cloud = camera_interface->GetLatestPointCloud(&timestamp);
    raw_depth = camera_interface->GetLatestImage(depth_type, &timestamp);
    if (!raw_cloud || !raw_depth)
      continue;

    camera_pose = robot_comm->GetCameraPose().cast<float>();

    bool err = fusion->ProcFrame(camera_pose, *raw_cloud, *raw_depth);
    if (!err)
      continue;

    Eigen::Isometry3f world_update;
    std::unique_lock<std::mutex> lock1(g_cloud_mutex);
    fusion->GetLatestFusedPointCloudAndWorldFrameUpdate(&g_fused_cloud,
                                                        &world_update);
    g_fused_cloud_ctr++;
    lock1.unlock();

    perception::VisualizePointCloudDrake(*g_fused_cloud, lcm,
        Eigen::Isometry3d::Identity(), "DRAKE_POINTCLOUD_FUSED");
  }
}

template <typename T>
bool IsIdentity(const Eigen::Transform<T, 3, Eigen::Isometry> &transform,
                T ang_thresh, T pos_thresh) {
  auto ang_err = Eigen::AngleAxis<T>(transform.linear());
  auto pos_err = transform.translation();

  return (std::fabs(ang_err.angle()) < ang_thresh &&
          pos_err.norm() < pos_thresh);
}

bool ExecuteGrasp(robot_bridge::RobotBridge &robot_comm,
                  const WorkCell& cell,
                  const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr &obj_cloud,
                  const Eigen::Isometry3d &grasp_pose,
                  double duration, lcm::LCM *lcm) {
  // Approach pregrasp, also have the icp thread correct the goal.
  double duration_move_pregrasp = duration;
  double z_above = 0.15;
  auto pre_grasp =
      Eigen::Translation3d(Eigen::Vector3d(0, 0, z_above)) * grasp_pose;
  auto status = robot_comm.MoveTool(pre_grasp, duration_move_pregrasp, Eigen::Vector6d::Constant(10000), false);

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformed_obj =
      obj_cloud->makeShared();

  auto updated_pre_grasp = pre_grasp;

  int last_ctr = 0;
  int iterations = 0;
  while (true) {
    // Get the current fused cloud.
    std::unique_lock<std::mutex> lock1(g_cloud_mutex);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr fused_cloud =
        g_fused_cloud;
    int ctr = g_fused_cloud_ctr;
    lock1.unlock();

    if (ctr == last_ctr) {
      continue;
    }
    last_ctr = ctr;
    iterations++;

    Eigen::Isometry3f update;
    double score = perception::AlignCloud<pcl::PointXYZRGBNormal>(
        transformed_obj,
        perception::DownSample<pcl::PointXYZRGBNormal>(fused_cloud, 0.004),
        0.05, transformed_obj, &update);
    if (score < 0)
      return false;

    perception::VisualizePointCloudDrake(*transformed_obj, lcm,
                                         Eigen::Isometry3d::Identity(),
                                         "DRAKE_POINTCLOUD_TRANSFORMED_OBJ");

    // Update grasp
    updated_pre_grasp = update.cast<double>() * updated_pre_grasp;
    robot_comm.UpdateToolGoal(updated_pre_grasp);

    auto robot_v = robot_comm.GetJointVelocityRadians();
    if (IsIdentity<float>(update, 0.005, 0.003) &&
        robot_v.norm() < 0.05 &&
        iterations >= 5) {
      transformed_obj->clear();
      perception::VisualizePointCloudDrake(*transformed_obj, lcm,
                                         Eigen::Isometry3d::Identity(),
                                         "DRAKE_POINTCLOUD_TRANSFORMED_OBJ");
      break;
    }
  }

  g_skip_fusion = true;
  do {
    status = robot_comm.GetRobotMotionStatus();
  } while (status == robot_bridge::MotionStatus::EXECUTING);

  if (status == robot_bridge::MotionStatus::ERR_STUCK) {
    std::cout << "ERRRR: stuck, either singularity or joint limit.\n";
    return false;
  }

  // at pregrasp, turn of icp update thread, and move straight down from the
  // current deiresd pose (have already been transformed by the icp thread.)
  double duration_move_grasp = 1.5;
  Eigen::Isometry3d X_WT_now = robot_comm.GetDesiredToolPose();
  Eigen::Isometry3d target = Eigen::Translation3d(Eigen::Vector3d(0, 0, -z_above - 0.03)) * X_WT_now;

  // Will grab the side of the bin
  if (CheckBinIntersection(target, 0.06, cell.X_bin, cell.bin_half_dim))
    return false;

  Eigen::Vector6d F_thresh = Eigen::Vector6d::Constant(10000);
  F_thresh[5] = 30;
  status = robot_comm.MoveTool(
      target, duration_move_grasp, F_thresh, true);
  if (status != robot_bridge::MotionStatus::DONE) {
    std::cout << "ERRRR: " << (int)status << "\n";
  }

  // Close gripper.
  bool grasped = robot_comm.CloseGripper();

  // Lift obj.
  if (grasped) {
    double duration_lift_up = 1.5;
    Eigen::Isometry3d cur_pose = robot_comm.GetToolPose();
    cur_pose.translation()(2) += 0.3;
    robot_comm.MoveTool(cur_pose, duration_lift_up, Eigen::Vector6d::Constant(10000), true);
    // Check grasp again in case we dropped it.
    grasped = robot_comm.CheckGrasp();
  }

  return grasped;
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr
GetGraspObj(const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr &input,
            const Eigen::Isometry3d &grasp_pose) {
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr obj =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
  for (size_t i = 0; i < input->size(); i++) {
    Eigen::Vector3d pt(input->points[i].x, input->points[i].y,
                       input->points[i].z);
    if ((pt - grasp_pose.translation()).norm() < 0.09) {
      obj->push_back(input->points[i]);
    }
  }
  obj = perception::SOROutlierRemoval<pcl::PointXYZRGBNormal>(obj);
  return perception::DownSample<pcl::PointXYZRGBNormal>(obj, 0.004);
}

bool IsTableClear(
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr& grasp_cloud,
    lcm::LCM* lcm) {

  std::cout << "Grasp cloud size: " << grasp_cloud->size() << "\n\n";
  if (grasp_cloud->size() < 500)
    return true;

  auto bin_minus_bottom =
      perception::SubtractTable<pcl::PointXYZRGBNormal>(grasp_cloud, 0.01);
  std::cout << "Grasp cloud size: " << bin_minus_bottom->size() << "\n\n";
  return bin_minus_bottom->size() < 500;
}



int main(int argc, char **argv) {
  lcm::LCM lcm;
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      IIWA_MODEL_PATH, drake::multibody::joints::kFixed, nullptr, &tree);

  // Setup camera and gripper frames.
  Eigen::Isometry3d X_7C = Eigen::Isometry3d::Identity();
  // to rgb
  /*
  X_7C.translation() = Eigen::Vector3d(-0.012065, 0.04737, 0.135);
  X_7C =
      Eigen::AngleAxisd((-22. + 90.) * M_PI / 180, Eigen::Vector3d::UnitZ()) *
      X_7C * Eigen::AngleAxisd(12.71 * M_PI / 180, Eigen::Vector3d::UnitX());
  */
  X_7C.matrix() <<
        0.3826,    -0.880474,      0.27997,   -0.0491369,
        0.923914,     0.364806,    -0.115324,   0.00836689,
        -0.000594751,     0.302791,     0.953057,     0.135499,
                   0,            0,            0,            1;

  RealSenseSR300 camera_interface;

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
  robot_comm.OpenGripper();

  // Streaming loop.
  /*
  std::thread image_streaming_thread(perception::StreamImagesAsJpgLoop,
                                     &camera_interface, color_type, depth_type,
                                     "RGB_IMG", "DEPTH_IMG", &lcm);
  */

  // Cloud loop.
  std::function<Eigen::Vector2f(const Eigen::Vector3f &)> proj_func =
      std::bind(&rgbd_bridge::RealSenseSR300::Project, &camera_interface, depth_type, std::placeholders::_1);
  perception::PointCloudFusion fusion(proj_func, 0.002);
  std::thread cloud_fusion_thread(CloudThread, &robot_comm, &camera_interface,
                                  depth_type, &fusion, &lcm);

  // Grasp generator.
  AntiPodalGraspPlanner grasp_planner(GRASP_PARAM_PATH);

  // Task related magic numbers.
  WorkCell front_cell;
  WorkCell right_cell;

  // Bin template stuff. This model is scanned in front of the robot. close
  // to front cell.
  {
    front_cell.q_prep_deg <<
      0, 11, 0, -86, 0, 82, 22;
    front_cell.q_scan_left_deg <<
      -39, 63, -2, -40, 33, 108, -37;
    front_cell.q_scan_mid_deg <<
      0, 24, -2, -68, 1, 86, 17;
      //20, -22, -2, -118, -23, 54, 48;
    front_cell.q_scan_right_deg <<
      44, 71, -2, -30, -31, 107, 17;
    front_cell.q_place_deg <<
      -90, 23, 0, -92, 0, 63, 22;
    front_cell.X_bin = Eigen::Translation3d(Eigen::Vector3d(0.53, 0, 0.03));
    front_cell.world_rot = Eigen::Isometry3d::Identity();

    right_cell.q_prep_deg <<
      -90, 11, 0, -86, 0, 82, 22;
    right_cell.q_scan_left_deg <<
      -129, 63, -2, -40, 33, 108, -37;
    right_cell.q_scan_mid_deg <<
      -90, 24, -2, -68, 1, 86, 17;
      //-70, -22, -2, -118, -23, 54, 48;
    right_cell.q_scan_right_deg <<
      -46, 71, -2, -30, -31, 107, 17;
    right_cell.q_place_deg <<
      0, 23, 0, -92, 0, 63, 22;
    right_cell.world_rot = Eigen::Isometry3d(
        Eigen::AngleAxisd(-M_PI / 2., Eigen::Vector3d::UnitZ()));
    right_cell.X_bin = right_cell.world_rot * Eigen::Translation3d(Eigen::Vector3d(0.53, 0, 0.01));
  }

  const WorkCell* cur_cell = &front_cell;

  if (argc > 1) {
    if (std::string(argv[1]).compare("f") == 0)
      cur_cell = &front_cell;
    else if (std::string(argv[1]).compare("r") == 0)
      cur_cell = &right_cell;
    else {
      std::cout << "1st argument has to be either \"f\" or \"r\"\n";
      exit(-1);
    }
  }

  robot_comm.MoveJointDegrees(cur_cell->q_prep_deg, 2, true);
  robot_comm.MoveJointDegrees(cur_cell->q_scan_left_deg, 2, true);
  bool start_from_left = true;

  bool pick_success = true;
  int fail_ctr = 0;
  double duration = 1.3;

  while (true) {
    usleep(5e5);

    // Clear old map.
    fusion.Init();
    pick_success = false;

    // Start merging.
    g_skip_fusion = false;

    // Scan.
    auto q_now = robot_comm.GetJointPositionDegrees();
    if ((q_now - cur_cell->q_scan_left_deg).norm() < 5) {
      // Left to right.
      robot_comm.MoveJointDegrees(
          {cur_cell->q_scan_mid_deg, cur_cell->q_scan_right_deg},
          {duration, duration}, true);
    } else if ((q_now - cur_cell->q_scan_right_deg).norm() < 5) {
      // Right to left.
      robot_comm.MoveJointDegrees(
          {cur_cell->q_scan_mid_deg, cur_cell->q_scan_left_deg},
          {duration, duration}, true);
    } else {
      assert(false);
    }

    // Get the latest fused cloud.
    std::unique_lock<std::mutex> lock1(g_cloud_mutex);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr fused_cloud =
        g_fused_cloud;
    lock1.unlock();

    // Cut workspace and minus bin
    auto grasp_cloud =
        perception::CutWithWorkSpaceConstraints<pcl::PointXYZRGBNormal>(
            fused_cloud,
            -cur_cell->bin_half_dim.cast<float>(),
            cur_cell->bin_half_dim.cast<float>() + Eigen::Vector3f(0, 0, 0.3),
            cur_cell->X_bin.cast<float>());
            // Eigen::Vector3f(0.42, -0.25, -0.0),
            // Eigen::Vector3f(0.63, 0.25, 0.3),
            // cur_cell->world_rot.cast<float>());
    // Filter out all black points..
    //grasp_cloud = perception::SubtractPointsByColor<pcl::PointXYZRGBNormal>(
    //    grasp_cloud, 100, 100, 100);
    // grasp_cloud = perception::SubtractTable<pcl::PointXYZRGBNormal>(grasp_cloud, 0.01);

    grasp_cloud = perception::DownSample<pcl::PointXYZRGBNormal>(grasp_cloud, 0.004);
    grasp_cloud = perception::SOROutlierRemoval<pcl::PointXYZRGBNormal>(grasp_cloud);

    perception::VisualizePointCloudDrake(*fused_cloud, &lcm,
                                         Eigen::Isometry3d::Identity(),
                                         "DRAKE_POINTCLOUD_BEFORE_CUT");
    perception::VisualizePointCloudDrake(*grasp_cloud, &lcm,
                                         Eigen::Isometry3d::Identity(),
                                         "DRAKE_POINTCLOUD_GRASP");
    if (IsTableClear(grasp_cloud, &lcm) || fail_ctr >= 5) {
      std::cout << "Switching bin!!!\n";
      if (cur_cell == &front_cell)
        cur_cell = &right_cell;
      else
        cur_cell = &front_cell;

      robot_comm.MoveJointDegrees(cur_cell->q_prep_deg, 2, true);
      robot_comm.MoveJointDegrees(cur_cell->q_scan_left_deg, 2, true);
      start_from_left = true;
      pick_success = true;
      fail_ctr = 0;
      continue;
    }

    pcl::io::savePCDFileASCII("grasp.pcd", *grasp_cloud);

    // Call grasp planner.
    grasp_planner.SetInputCloud(grasp_cloud);

    // Move to middle first non blocking.
    robot_comm.MoveJointDegrees(cur_cell->q_prep_deg, 2, false);

    // While moving, figure out what's the best grasp.
    std::vector<AntiPodalGrasp> all_grasps =
        grasp_planner.GenerateAntipodalGrasp();

    const AntiPodalGrasp* best_grasp =
        PickBestGrasp(*cur_cell,
                      all_grasps,
                      grasp_cloud);

    // Make sure we wait till the motion is done.
    robot_comm.WaitForRobotMotionCompletion();

    // Execute the grasp if there is a valid candidate.
    if (best_grasp) {
      Eigen::Isometry3d grasp_pose = best_grasp->hand_pose;
      // Extract the relative relationship between grasp_pose wrt to cloud.
      pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr obj_cloud =
          GetGraspObj(fused_cloud, grasp_pose);

      // Grab it.
      bool flag_grasp_success =
          ExecuteGrasp(robot_comm, *cur_cell, obj_cloud, grasp_pose,
                       duration, &lcm);

      if (flag_grasp_success) {
        // Transport to bin.
        robot_comm.MoveJointDegrees(cur_cell->q_place_deg, true);
        pick_success = true;
      } else {
        // Go to mid scan
        robot_comm.MoveJointDegrees(cur_cell->q_prep_deg, 1.5, true);
      }
    }

    // Reset gripper state.
    robot_comm.OpenGripper();

    if (pick_success) {
      robot_comm.MoveJointDegrees(
          {cur_cell->q_prep_deg, cur_cell->q_scan_left_deg},
          {duration, duration}, true);
      fail_ctr = 0;
    } else {
      fail_ctr++;
      if (start_from_left) {
        start_from_left = false;
        robot_comm.MoveJointDegrees(cur_cell->q_scan_right_deg, true);
      } else {
        robot_comm.MoveJointDegrees(cur_cell->q_scan_left_deg, true);
      }
    }
  }
  return 0;
}
