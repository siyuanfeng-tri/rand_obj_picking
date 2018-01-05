#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "robot_bridge/iiwa_controller.h"

#include "perception/perception.h"
#include "rgbd_bridge/real_sense_sr300.h"

#include <stdexcept>
#include <string>

// Usage, in build:
// ./gen_calib_img ../cfg/calibration_joints.txt
// This will then outputs 48 rgb img, and ir img, and a poses.txt that contains
// the link_7 frame poses.
// You then need to copy all those to a camera calibration repo
// https://github.com/robinzhoucmu/CameraCalibration.git
// e.g. dump those to ~/code/CameraCalibration/data/calibration_data
// and run /home/user/code/CameraCalibration/build/test_hand_eye
// which will tell you the extrinsics from link7 frame to rgb or ir
// depending on which set of images you use. You can switch between
// rgb and ir by changing the source..
// /home/user/code/CameraCalibration/demo_test_handeye.cpp ln 28
int main(int argc, char **argv) {
  if (argc != 2) {
    std::cout << "need path to posture file.\n";
    exit(-1);
  }

  lcm::LCM lcm;
  RigidBodyTree<double> tree;
  const std::string robot_model_path = std::string(MODEL_DIR) +
      "iiwa_description/urdf/iiwa14_polytope_collision.urdf";
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      robot_model_path, drake::multibody::joints::kFixed, nullptr, &tree);

  // Doesn't use this here.
  Eigen::Isometry3d X_7C = Eigen::Isometry3d::Identity();

  rgbd_bridge::RealSenseSR300 camera_interface;

  camera_interface.Start(
      {rgbd_bridge::ImageType::IR, rgbd_bridge::ImageType::RGB},
      rgbd_bridge::ImageType::IR);

  camera_interface.set_laser_projector_power(0);
  camera_interface.set_laser_projector_power(0);
  camera_interface.set_laser_projector_power(0);
  camera_interface.set_laser_projector_power(0);
  camera_interface.set_laser_projector_power(0);
  camera_interface.set_laser_projector_power(0);

  RigidBodyFrame<double> camera_frame(
      "Camera", tree.FindBody("iiwa_link_7"), X_7C);

  RigidBodyFrame<double> tool_frame("Tool",
                                    tree.FindBody("iiwa_link_7"),
                                    Eigen::Isometry3d::Identity());

  // This part is really hacked...
  std::ifstream in;
  in.open(argv[1], std::ifstream::in);

  std::vector<Eigen::VectorXd> q_setpoints(49, Eigen::VectorXd::Zero(7));
  for (int i = 0; i < 49; i++) {
    for (int j = 0; j < 7; j++) {
      in >> q_setpoints[i][j];
    }
    std::cout << q_setpoints[i].transpose() << "\n";
  }

  robot_bridge::IiwaController robot_comm(tree, tool_frame, camera_frame);
  robot_comm.Start();

  std::ofstream out;
  out.open("poses.txt", std::ofstream::out);

  uint64_t timestamp;
  cv::Mat tmp;
  int ctr = 0;
  Eigen::VectorXd v;
  camera_interface.set_laser_projector_power(0);

  robot_comm.MoveJointDegrees(q_setpoints[0], 3, true);

  for (size_t i = 0; i < q_setpoints.size(); i++) {
    if (i == 21)
      continue;

    const auto &q = q_setpoints[i];

    robot_comm.MoveJointDegrees(q, 0.8, true);
    usleep(5e5);

    auto ir = camera_interface.GetLatestImage(rgbd_bridge::ImageType::IR,
                                              &timestamp);
    auto depth = camera_interface.GetLatestImage(rgbd_bridge::ImageType::RGB,
                                                 &timestamp);
    auto pose = robot_comm.GetToolPose();

    ir->convertTo(tmp, CV_8UC1, 255. / (1 << 16), 0);
    perception::SendImageAsJpg(tmp, "DEPTH_IMG", &lcm);
    cv::imwrite(std::string("ir_") + std::to_string(ctr) + std::string(".jpg"),
                tmp);

    cv::cvtColor(*depth, tmp, CV_RGB2BGR);
    perception::SendImageAsJpg(tmp, "RGB_IMG", &lcm);
    cv::imwrite(
        std::string("rgb_") + std::to_string(ctr++) + std::string(".jpg"), tmp);

    out << pose.matrix() << "\n";
  }
  out.close();
  camera_interface.Stop();
  robot_comm.Stop();

  return 0;
}
