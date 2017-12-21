#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "robot_bridge/iiwa_controller.h"

int main() {

  lcm::LCM lcm;
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      IIWA_MODEL_PATH, drake::multibody::joints::kFixed, nullptr, &tree);

  // Setup camera and gripper frames.
  Eigen::Isometry3d X_7C = Eigen::Isometry3d::Identity();
  // to rgb
  X_7C.matrix() <<
        0.3826,    -0.880474,      0.27997,   -0.0491369,
        0.923914,     0.364806,    -0.115324,   0.00836689,
        -0.000594751,     0.302791,     0.953057,     0.135499,
                   0,            0,            0,            1;

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

  Eigen::VectorXd q = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd q1 = Eigen::VectorXd::Zero(7);
  q[1] = 45;
  q[3] = 1;
  q[5] = 45;

  q1 << 0, 40, -0, -77, -0, 63, -56;
  robot_comm.MoveJointDegrees(q1, 2, true);

  robot_bridge::RobotState robot_state(&tree, &tool_frame);
  Eigen::Isometry3d X0;

  robot_comm.GetRobotState(&robot_state);
  X0 = robot_state.get_X_WT();

  /*
  Eigen::Vector6d gains0 = Eigen::Vector6d::Ones();
  gains0.head<3>().setZero();
  for (int i = 0; i < 3; i++) {
    robot_comm.MoveTool(Eigen::Translation3d(Eigen::Vector3d(-0.4, 0, 0.)) * X0, 3, true);
    std::cout << robot_comm.GetJointPositionDegrees().transpose() << "\n";

    robot_comm.MoveTool(Eigen::Translation3d(Eigen::Vector3d(0.4, 0, 0.)) * X0, gains0, 3, true);
  }
  */

  robot_comm.MoveTool(
      X0 * Eigen::AngleAxisd(M_PI/2., Eigen::Vector3d::UnitY()), 1, true);

  while(true)
    ;
  robot_comm.Stop();

  return 0;
}
