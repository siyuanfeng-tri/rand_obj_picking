#include "robot_bridge/iiwa_controller.h"
#include "grasp_gen/anti_podal_grasp.h"
#include "perception/point_cloud_fusion.h"
#include "perception/perception.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"

#include "rgbd_bridge/real_sense_sr300.h"
#include <pcl/filters/passthrough.h>

#include <atomic>
#include <string>
#include <stdexcept>
#include <opencv2/opencv.hpp>

using rgbd_bridge::RealSenseSR300;

bool CheckSimilarityGrasp(Eigen::Isometry3d prev_pose,
		Eigen::Isometry3d cur_pose, double norm_threshold = 0.075) {
	Eigen::Vector3d diff_translation =
		prev_pose.translation() - cur_pose.translation();
	if (diff_translation.norm() > norm_threshold) {
		return true;
	}
	else {
		return false;
	}
}

bool CheckReachableConstraint(const Eigen::Isometry3d& pose) {
	const double max_x = 0.8;
	const double min_x = 0.40;
	double pose_x = pose.translation()(0);
  double yaw_angle = atan2(-pose.matrix()(0,1), pose.matrix()(0,0));
	if (pose_x > min_x && pose_x < max_x
			&& fabs(yaw_angle) < M_PI * 3.0 / 4.0 ) {
		std::cout << "The grasp is reachable" << std::endl;
		return true;
	} else {
		std::cout << "The grasp is NOT reachable" << std::endl;
    std::cout << pose.translation().transpose() << ", yaw: " << yaw_angle << "\n";
		return false;
	}
}

AntiPodalGrasp PickBestGrasp(std::vector<AntiPodalGrasp> all_grasps) {
	assert(all_grasps.size() >= 1);
	int rand_max_N = std::min(int(all_grasps.size()), 100);
	bool flag_reachable = false;
	int total_sample_trials = 100;
	int sample_trial = 0;
	int grasp_to_choose_id = 0;
	while (sample_trial < total_sample_trials && !flag_reachable) {
		grasp_to_choose_id = rand() % rand_max_N;
		Eigen::Isometry3d grasp_pose = all_grasps[grasp_to_choose_id].hand_pose;
		flag_reachable = CheckReachableConstraint(grasp_pose);
		sample_trial++;
	}
	return all_grasps[grasp_to_choose_id];
}

std::mutex g_cloud_mutex;
Eigen::Isometry3f g_world_update;
pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr g_fused_cloud;
std::atomic<bool> g_skip_fusion{false};
void CloudThread(robot_bridge::RobotBridge* robot_comm,
                 const rgbd_bridge::RealSenseSR300* camera_interface,
                 const rgbd_bridge::ImageType depth_type,
                 perception::PointCloudFusion* fusion,
                 lcm::LCM* lcm) {
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
    /*
    pcl::transformPointCloud(*raw_cloud, *points, camera_pose);
    perception::VisualizePointCloudDrake(*points, lcm);
    usleep(1e5);
    */

    //bool err = fusion->ProcFrame(camera_pose, *camera_interface,
    //    *raw_cloud, *raw_depth, depth_type);
    bool err = fusion->ProcFrame(camera_pose, *raw_cloud, *raw_depth);
    if (!err)
      continue;

    std::unique_lock<std::mutex> lock1(g_cloud_mutex);
    fusion->GetLatestFusedPointCloudAndWorldFrameUpdate(&g_fused_cloud, &g_world_update);
    lock1.unlock();

    // pcl::io::savePCDFileASCII("cloud_with_normal" + std::to_string(ctr++) + ".pcd", *g_fused_cloud);

    perception::VisualizePointCloudDrake(*g_fused_cloud, lcm);
    // update grasp..
    robot_comm->UpdateToolGoal(g_world_update.cast<double>());
  }
}

bool ExecuteGrasp(robot_bridge::RobotBridge& robot_comm,
    const Eigen::VectorXd& q_prep_deg,
    const Eigen::Isometry3d& grasp_pose,
		const rgbd_bridge::RealSenseSR300& camera_interface,
    double duration) {

  robot_comm.MoveJointDegrees(q_prep_deg, 2, true);
  std::cout << "Picked grasp pose(hand frame)\n";
  std::cout << grasp_pose.matrix() << std::endl;

  // Approach pregrasp, also have the icp thread correct the goal.
  double duration_move_pregrasp = duration;
  double z_above = 0.15;
  auto pre_grasp = Eigen::Translation3d(Eigen::Vector3d(0, 0, z_above)) * grasp_pose;
  robot_comm.MoveTool(pre_grasp, duration_move_pregrasp, 10000, true);
  g_skip_fusion = true;

  // at pregrasp, turn of icp update thread, and move straight down from the
  // current deiresd pose (have already been transformed by the icp thread.)
  double duration_move_grasp = duration;
  Eigen::Isometry3d X_WT_now = robot_comm.GetDesiredToolPose();
  robot_comm.MoveTool(
      Eigen::Translation3d(Eigen::Vector3d(0, 0, -z_above)) *
      X_WT_now, duration_move_grasp, 15, true);

  // Close gripper.
  bool grasped = robot_comm.CloseGripper();
  return grasped;
}

int main(int argc, char** argv) {
	lcm::LCM lcm;
	RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      IIWA_MODEL_PATH, drake::multibody::joints::kFixed, nullptr, &tree);

  Eigen::Isometry3d X_7C = Eigen::Isometry3d::Identity();
  // to rgb
  X_7C.translation() = Eigen::Vector3d(-0.012065, 0.04737, 0.135);
  X_7C =
      Eigen::AngleAxisd((-22. + 90.) * M_PI / 180, Eigen::Vector3d::UnitZ()) *
      X_7C *
      Eigen::AngleAxisd(16.71 * M_PI / 180, Eigen::Vector3d::UnitX());

  /*
  X_7C.matrix() <<
    -0.3968,    0.9179,    0.0090,   -0.0415,
    -0.9179,   -0.3968,   -0.0044,    0.0212,
    -0.0005,   -0.0100,    1.0000,    0.1332,
    0,         0,         0,    1;
  */

  std::cout << "X_7C:\n" << X_7C.matrix() << "\n";

  RealSenseSR300 camera_interface;
  /*
  camera_interface.set_focal_length_x(rgbd_bridge::ImageType::DEPTH, 481.634496904251);
  camera_interface.set_focal_length_y(rgbd_bridge::ImageType::DEPTH, 482.5884407032157);
  camera_interface.set_principal_point_x(rgbd_bridge::ImageType::DEPTH, 321.0790258817788);
  camera_interface.set_principal_point_y(rgbd_bridge::ImageType::DEPTH, 245.2850786716308);
  camera_interface.set_focal_length_x(rgbd_bridge::ImageType::IR, 481.634496904251);
  camera_interface.set_focal_length_y(rgbd_bridge::ImageType::IR, 482.5884407032157);
  camera_interface.set_principal_point_x(rgbd_bridge::ImageType::IR, 321.0790258817788);
  camera_interface.set_principal_point_y(rgbd_bridge::ImageType::IR, 245.2850786716308);

  camera_interface.set_focal_length_x(rgbd_bridge::ImageType::RGB, 618.8411523746104);
  camera_interface.set_focal_length_y(rgbd_bridge::ImageType::RGB, 620.3575322440417);
  camera_interface.set_principal_point_x(rgbd_bridge::ImageType::RGB, 312.7944888695696);
  camera_interface.set_principal_point_y(rgbd_bridge::ImageType::RGB, 235.5000184550021);
  camera_interface.set_focal_length_x(rgbd_bridge::ImageType::RECT_RGB, 618.8411523746104);
  camera_interface.set_focal_length_y(rgbd_bridge::ImageType::RECT_RGB, 620.3575322440417);
  camera_interface.set_principal_point_x(rgbd_bridge::ImageType::RECT_RGB, 312.7944888695696);
  camera_interface.set_principal_point_y(rgbd_bridge::ImageType::RECT_RGB, 235.5000184550021);
  camera_interface.set_focal_length_x(rgbd_bridge::ImageType::RECT_RGB_ALIGNED_DEPTH, 618.8411523746104);
  camera_interface.set_focal_length_y(rgbd_bridge::ImageType::RECT_RGB_ALIGNED_DEPTH, 620.3575322440417);
  camera_interface.set_principal_point_x(rgbd_bridge::ImageType::RECT_RGB_ALIGNED_DEPTH, 312.7944888695696);
  camera_interface.set_principal_point_y(rgbd_bridge::ImageType::RECT_RGB_ALIGNED_DEPTH, 235.5000184550021);
  */

  //const rgbd_bridge::ImageType depth_type = rgbd_bridge::ImageType::DEPTH;
  //const rgbd_bridge::ImageType color_type = rgbd_bridge::ImageType::DEPTH_ALIGNED_RGB;
  const rgbd_bridge::ImageType depth_type = rgbd_bridge::ImageType::RECT_RGB_ALIGNED_DEPTH;
  const rgbd_bridge::ImageType color_type = rgbd_bridge::ImageType::RECT_RGB;
  const std::vector<rgbd_bridge::ImageType> channels = {color_type, depth_type};
  camera_interface.Start({color_type, depth_type, rgbd_bridge::ImageType::DEPTH}, color_type);

  RigidBodyFrame<double> camera_frame("Camera", tree.FindBody(robot_bridge::kEEName),
  		X_7C);

  const Eigen::Isometry3d X_7T =
      Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.24)) *
      Eigen::AngleAxis<double>(-22. / 180. * M_PI, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d::UnitY());
	RigidBodyFrame<double> tool_frame("Tool", tree.FindBody(robot_bridge::kEEName),
  		X_7T);
  const Eigen::Isometry3d X_TC = X_7T.inverse() * X_7C;

  std::cout << "X_7T:\n" << X_7T.matrix() << "\n";

  robot_bridge::IiwaController robot_comm(tree, tool_frame, camera_frame);
  robot_comm.Start();
	robot_comm.OpenGripper();

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr original_cloud =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();

  // Streaming loop.
  //std::thread image_streaming_thread(perception::StreamImagesAsJpgLoop,
  //    &camera_interface, color_type, depth_type, "RGB_IMG", "DEPTH_IMG", &lcm);

  // Cloud loop.
  std::function<Eigen::Vector2f(const Eigen::Vector3f &)> proj_func =
      std::bind(&rgbd_bridge::RealSenseSR300::Project, &camera_interface, depth_type, std::placeholders::_1);
  perception::PointCloudFusion fusion(proj_func, 0.002);
  std::thread cloud_fusion_thread(
      CloudThread, &robot_comm, &camera_interface, depth_type, &fusion, &lcm);

	double duration = 10.0;
	if (argc > 1) {
		duration = std::atof(argv[1]);
	}

  RigidBodyFrame<double> gaze_frame("gaze", tree.FindBody(robot_bridge::kEEName),
      Eigen::Isometry3d::Identity());

  // Grasp generator.
	AntiPodalGraspPlanner grasp_planner(GRASP_PARAM_PATH);

  // Waypoints.
  Eigen::VectorXd q_place_deg(7);
  q_place_deg << -72.883,29.6285,-13.3159,-92.5593,7.56961,58.9519,-66.4744;
 	Eigen::VectorXd q_prep_deg(7);
  q_prep_deg << 1.75373,11.7494,-2.09317,-86.2383,0.42104,82.5027,21.6225;

  // Scan waypoints
  std::vector<Eigen::VectorXd> q_scans(4, Eigen::VectorXd(7));
  // Left
  q_scans[0] << -39, 63, -2, -40, 33, 108, -37;
  // Mid front
  q_scans[1] << 5, 63, -2, -20, -8, 111, 0;
  // Right
  q_scans[2] << 44, 71, -2, -30, -31, 107, 17;
  // Mid back
  q_scans[3] << 5, 8, -2, -119, -6, 41, 16;
  for (size_t i = 0; i < q_scans.size(); i++) {
    q_scans[i] = q_scans[i] * M_PI / 180.;
  }
  const Eigen::VectorXd& q_scan_left = q_scans[0];
  const Eigen::VectorXd& q_scan_right = q_scans[2];
  const Eigen::VectorXd& q_scan_mid_front = q_scans[1];
  const Eigen::VectorXd& q_scan_mid_back = q_scans[3];

  robot_comm.MoveJointDegrees(q_prep_deg, 2, true);
  robot_comm.MoveJointRadians(q_scan_left, 2, true);

	while (true) {
    camera_interface.set_mode(
      //rs_ivcam_preset::RS_IVCAM_PRESET_BACKGROUND_SEGMENTATION);
      rs_ivcam_preset::RS_IVCAM_PRESET_SHORT_RANGE);

    // Clear old map.
    fusion.Init();
    // Start merging.
    g_skip_fusion = false;
    // Scan.
    auto q_now = robot_comm.GetJointPositionRadians();
    std::cout << "q_now: " << q_now.transpose() << "\n";
    std::cout << "q_diff_left: " << (q_now - q_scan_left).transpose() << "\n";
    std::cout << "q_diff_right: " << (q_now - q_scan_right).transpose() << "\n";

    if ((q_now - q_scan_left).norm() < 0.1) {
      robot_comm.MoveJointRadians({q_scan_mid_front, q_scan_right},
          {duration, duration}, true);
    } else if ((q_now - q_scan_right).norm() < 0.1) {
      robot_comm.MoveJointRadians({q_scan_mid_back, q_scan_left},
          {duration, duration}, true);
    } else {
      assert(false);
    }

    std::unique_lock<std::mutex> lock1(g_cloud_mutex);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr fused_cloud = g_fused_cloud;
    lock1.unlock();

    // Cut workspace and minus table
    auto cut_cloud = perception::CutWithWorkSpaceConstraints<pcl::PointXYZRGBNormal>(
        fused_cloud,
        Eigen::Vector3f(0.4, -0.3, -0.02),
        Eigen::Vector3f(0.95, 0.3, 0.3));
    cloud = perception::OutlierRemoval<pcl::PointXYZRGBNormal>(cut_cloud, 50);
    perception::SubtractTable<pcl::PointXYZRGBNormal>(cloud, 0.005);

    perception::VisualizePointCloudDrake(*cloud, &lcm);
	  grasp_planner.SetInputCloud(cloud);

    // perception::VisualizePointCloudAndNormal(cloud);

	  std::vector<AntiPodalGrasp> all_grasps = grasp_planner.GenerateAntipodalGrasp();

    camera_interface.set_mode(
        rs_ivcam_preset::RS_IVCAM_PRESET_SHORT_RANGE);

    // If no grasps
    if (!all_grasps.empty()) {
      AntiPodalGrasp best_grasp = PickBestGrasp(all_grasps);
      Eigen::Isometry3d grasp_pose = best_grasp.hand_pose;
      bool flag_reachable = CheckReachableConstraint(grasp_pose);
      if (flag_reachable) {
        bool flag_grasp_success = ExecuteGrasp(robot_comm, q_prep_deg, grasp_pose, camera_interface, duration);

        if (flag_grasp_success) {
          double duration_lift_up = 1.5;
          Eigen::Isometry3d cur_pose = robot_comm.GetToolPose();
          cur_pose.translation()(2) += 0.3;
          robot_comm.MoveTool(cur_pose, duration_lift_up, 1000, true);
          // Check grasp again in case we dropped it.
          flag_grasp_success = robot_comm.CheckGrasp();
        }

        if (flag_grasp_success) {
          // Transport to bin.
          robot_comm.MoveJointDegrees(q_place_deg, true);
        } else {
          // Go to mid scan
          robot_comm.MoveJointDegrees(q_prep_deg, 1.5, true);
        }
        robot_comm.OpenGripper();
        // Done 1 iteration.
        robot_comm.MoveJointRadians(q_scan_left, 1.5, true);
      }
		}
	}
	return 0;
}

