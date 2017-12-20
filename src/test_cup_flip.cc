#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "robot_bridge/iiwa_controller.h"

// Cup frame: handle is +x, opening is z. origin at the cup bottom, center.
void FlipCup(
    const Eigen::Isometry3d& cup_pose,
    const Eigen::Isometry3d& grasp_in_cup_frame,
    robot_bridge::RobotBridge& robot_comm) {
  const double kLiftZ = 0.1;
  const double kLinMotionDt = 1.5;
  const double kRotMotionDt = 3;

  Eigen::Isometry3d X0 =
      Eigen::Translation3d(Eigen::Vector3d(0, 0, kLiftZ)) *
      cup_pose * grasp_in_cup_frame;

  // Pose
  robot_comm.MoveTool(X0, kLinMotionDt, Eigen::Vector6d::Constant(50), true);

  // go down
  X0 = Eigen::Translation3d(Eigen::Vector3d(0, 0, -kLiftZ)) * X0;
  robot_comm.MoveTool(X0, kLinMotionDt, Eigen::Vector6d::Constant(50), true);

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
  Eigen::Vector6d F_thresh = Eigen::Vector6d::Constant(50);
  F_thresh[5] = 20;
  robot_comm.MoveTool(X0, kLinMotionDt, F_thresh, true);

  // drop
  robot_comm.OpenGripper();

  // rewind
  X0 = X0 * Eigen::AngleAxis<double>(-M_PI / 2., Eigen::Vector3d::UnitY());
  robot_comm.MoveTool(X0, kRotMotionDt, Eigen::Vector6d::Constant(50), true);
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

  // lift
  X0 = Eigen::Translation3d(Eigen::Vector3d(0, 0, kLiftZ)) * X0;
  robot_comm.MoveTool(X0, kLinMotionDt, Eigen::Vector6d::Constant(50), true);

}

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cout << "need to input num of plates.\n";
    exit(-1);
  }

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

  robot_bridge::IiwaController robot_comm(tree, tool_frame, camera_frame);
  robot_comm.Start();

  Eigen::VectorXd q0(7);
  q0 << 41.1795,  59.8544, -72.0594,  -94.348,  15.4208,  47.2948,   92.208;

  // robot_bridge::RobotState robot_state(&tree, &tool_frame);
  // Eigen::Isometry3d X0 = Eigen::Isometry3d::Identity();

  // Reset.
  robot_comm.OpenGripper();
  robot_comm.MoveJointDegrees(q0, 2, true);

  std::cout << "press enter to cont.\n";
  getchar();

  Eigen::Isometry3d grasp_in_cup_frame =
      Eigen::Translation3d(Eigen::Vector3d(0, 0, 0.04)) *
      Eigen::AngleAxis<double>(-M_PI / 4., Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxis<double>(-M_PI / 4., Eigen::Vector3d::UnitY());

  Eigen::Isometry3d cup_in_world =
      Eigen::Translation3d(Eigen::Vector3d(0.55, -0.25, 0.0)) *
      Eigen::AngleAxis<double>(-M_PI / 2., Eigen::Vector3d::UnitZ());

  FlipCup(cup_in_world, grasp_in_cup_frame, robot_comm);

  while(true) {
    ; //std::cout << robot_comm.GetJointPositionDegrees().transpose() << "\n";
  }

  return 0;
}


