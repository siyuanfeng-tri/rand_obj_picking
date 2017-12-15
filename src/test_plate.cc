#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "robot_bridge/iiwa_controller.h"

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
  RigidBodyFrame<double> camera_frame(
      "Camera", tree.FindBody(robot_bridge::kEEName), X_7C);

  const Eigen::Isometry3d X_7T =
      Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.24)) *
      Eigen::AngleAxis<double>(-22. / 180. * M_PI, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d::UnitY()) *
      Eigen::Translation<double, 3>(Eigen::Vector3d(0, -0.05, 0));
  RigidBodyFrame<double> tool_frame("Tool",
                                    tree.FindBody(robot_bridge::kEEName), X_7T);

  robot_bridge::IiwaController robot_comm(tree, tool_frame, camera_frame);
  robot_comm.Start();

  Eigen::VectorXd q0(7);
  q0 << -80, 40, -0, -77, -0, 63, -56;
  Eigen::VectorXd q1(7);
  q1 << -76, 33, -8, -47, 5, 99, -58;
  Eigen::VectorXd q2(7);
  q2 << 0, 33, -8, -47, 5, 99, -58;

  robot_bridge::RobotState robot_state(&tree, &tool_frame);
  Eigen::Isometry3d X0;

  for (int i = 0; i < std::atoi(argv[1]); i++) {
    // Reset.
    robot_comm.MoveJointDegrees(q0, true);

    // Go straight down.
    robot_comm.MoveStraightUntilTouch(
        Eigen::Vector3d::UnitZ(), -0.1,
        Eigen::Vector3d(100, 100, 15), true);

    // Push in.
    robot_comm.MoveStraightUntilTouch(
        Eigen::Vector3d::UnitY(), 0.03,
        Eigen::Vector3d(100, 20, 50), true);

    // Lift up.
    robot_comm.GetRobotState(&robot_state);
    X0 = robot_state.get_X_WT();
    X0.linear().setIdentity();
    X0.translation()[1] += 0.005;
    X0.translation()[2] += 0.02;
    robot_comm.MoveTool(X0, 1, Eigen::Vector6d::Constant(50), true);

    // Curl in.
    robot_comm.GetRobotState(&robot_state);
    X0 = robot_comm.GetDesiredToolPose();
    X0.linear() = Eigen::AngleAxis<double>(35. / 180. * M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    X0.translation()[1] += 0.01;
    robot_comm.MoveTool(X0, 1, Eigen::Vector6d::Constant(50), true);

    // Go in more.
    // X0 = robot_comm.GetDesiredToolPose();
    // X0 = X0 * Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, -0.02));
    // robot_comm.MoveTool(X0, 1, Eigen::Vector6d::Constant(50), true);

    // Grab.
    robot_comm.CloseGripper();

    // Lift.
    X0 = robot_comm.GetDesiredToolPose();
    X0.translation()[2] += 0.1;
    robot_comm.MoveTool(X0, 1, Eigen::Vector6d::Constant(50), true);
    robot_comm.MoveJointDegrees(q1, 2, true);

    // Reset.
    robot_comm.MoveJointDegrees(q2, true);

    // Drop.
    robot_comm.OpenGripper();
  }

  robot_comm.Stop();

  return 0;
}


