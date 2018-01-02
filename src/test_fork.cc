#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "perception/perception.h"
#include "perception/point_cloud_fusion.h"
#include "rgbd_bridge/real_sense_sr300.h"
#include "robot_bridge/iiwa_controller.h"
#include "robot_bridge/iiwa_controller.h"

#include "util.h"

std::atomic<bool> run_img_loop{true};

void mouse_click(int event, int x, int y, int flags, void *userdata) {
  std::vector<Eigen::Vector2i>* points =
      static_cast<std::vector<Eigen::Vector2i>*>(userdata);

  if (event == cv::EVENT_LBUTTONDOWN) {
    std::cout << "(x, y): " << x << ", " << y << "\n";
    if (points->size() < 2) {
      points->push_back(Eigen::Vector2i(x, y));
      if (points->size() == 2)
        run_img_loop = false;
    }
  }
}

void image_loop(const rgbd_bridge::RGBDSensor *driver,
                std::vector<Eigen::Vector2i>* points) {
  uint64_t timestamp;
  points->clear();

  cv::Mat tmp;
  while (run_img_loop) {
    auto img = driver->GetLatestImage(rgbd_bridge::ImageType::RECT_RGB,
                                      &timestamp);
    if (img) {
      cv::cvtColor(*img, tmp, CV_RGB2BGR);
      cv::imshow("IMG", tmp);
      cv::waitKey(5);
    }
  }

  cv::imwrite("img.jpg", tmp);
}

/*
std::vector<Eigen::Vector2i> Get2Points(const rgbd_bridge::RealSenseSR300 *real_sense) {
  std::vector<Eigen::Vector2i> points;
  cv::namedWindow("IMG");
  std::thread img_thread(image_loop, real_sense, &points);
  cv::setMouseCallback("IMG", mouse_click, &points);

  img_thread.join();

  return points;
}
*/

std::vector<Eigen::Vector2i> Get2Points(const rgbd_bridge::RealSenseSR300 *real_sense) {
  uint64_t timestamp;

  auto img = real_sense->GetLatestImage(rgbd_bridge::ImageType::RECT_RGB,
                                    &timestamp);
  while (!img) {
    img = real_sense->GetLatestImage(rgbd_bridge::ImageType::RECT_RGB,
                                    &timestamp);
  }

  cv::Mat hsv, tmp1;
  cv::cvtColor(*img, hsv, CV_RGB2HSV);
  cv::cvtColor(*img, tmp1, CV_RGB2GRAY);

  std::vector<int> u, v;

  for (int i = 0; i < hsv.rows; i++) {
    for (int j = 0; j < hsv.cols; j++) {
      uint16_t h = hsv.at<cv::Vec3b>(i, j)[0] * 2;
      // green
      if ((h > 50 && h < 200) || (j < 110 && i < 220)) {
        tmp1.at<uint8_t>(i, j) = 0;
      } else {
        tmp1.at<uint8_t>(i, j) = 255;
        u.push_back(j);
        v.push_back(i);
      }
    }
  }

  cv::imshow("IMG", tmp1);
  cv::waitKey(0);

  cv::Mat pca_data(u.size(), 2, CV_64FC1);
  for (size_t i = 0; i < u.size(); i++) {
    pca_data.at<double>(i, 0) = u.at(i);
    pca_data.at<double>(i, 1) = v.at(i);
  }

  cv::PCA pca_analysis(pca_data, cv::Mat(), CV_PCA_DATA_AS_ROW);

  cv::Point cntr(
      static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
      static_cast<int>(pca_analysis.mean.at<double>(0, 1)));

  std::cout << pca_analysis.mean.at<double>(0, 0) << ", "
            << pca_analysis.mean.at<double>(0, 1) << "\n";

  std::vector<cv::Point2d> eigen_vecs(2);
  std::vector<double> eigen_val(2);
  for (int i = 0; i < 2; ++i) {
    eigen_vecs[i] =
        cv::Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                    pca_analysis.eigenvectors.at<double>(i, 1));
    eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);

    std::cout << pca_analysis.eigenvectors.at<double>(i, 0) << ", "
              << pca_analysis.eigenvectors.at<double>(i, 1) << "\n";

    std::cout << "val: " << eigen_val[i] << "\n";
  }

  cv::Point p0(
      static_cast<int>(pca_analysis.mean.at<double>(0, 0) + 30 * eigen_vecs[0].x),
      static_cast<int>(pca_analysis.mean.at<double>(0, 1) + 30 * eigen_vecs[0].y));
  cv::Point p1(
      static_cast<int>(pca_analysis.mean.at<double>(0, 0) - 30 * eigen_vecs[0].x),
      static_cast<int>(pca_analysis.mean.at<double>(0, 1) - 30 * eigen_vecs[0].y));

  std::vector<Eigen::Vector2i> hahaha;
  hahaha.push_back(Eigen::Vector2i(p0.x, p0.y));
  hahaha.push_back(Eigen::Vector2i(p1.x, p1.y));

  cv::circle(tmp1, cntr, 3, cv::Scalar(100), 2);
  cv::circle(tmp1, p0, 3, cv::Scalar(100), 2);
  cv::circle(tmp1, p1, 3, cv::Scalar(100), 2);
  cv::imshow("IMG", tmp1);
  cv::waitKey(0);

  return hahaha;
}

Eigen::Vector3d to3d(
    const rgbd_bridge::RealSenseSR300& camera,
    const rgbd_bridge::ImageType type,
    const Eigen::Isometry3d& X_WC,
    const Eigen::Vector2i& uv) {
  Eigen::Vector3d pt0 = X_WC * camera.Deproject(type, uv, 0.5).cast<double>();
  Eigen::Vector3d pt1 =  X_WC * camera.Deproject(type, uv, 1).cast<double>();

  Eigen::Vector3d dir = (pt1 - pt0).normalized();

  double mag = -pt0[2] / dir[2];
  return pt0 + mag * dir;
}

int main() {
  lcm::LCM lcm;
  RigidBodyTree<double> tree;
  const std::string robot_model_path = std::string(MODEL_DIR) +
      "iiwa_description/urdf/iiwa14_polytope_collision.urdf";
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      robot_model_path, drake::multibody::joints::kFixed, nullptr, &tree);
  // Setup camera and gripper frames.
  Eigen::Isometry3d X_7C = Eigen::Isometry3d::Identity();
  // to rgb
  X_7C.matrix() <<
        0.3826,    -0.880474,      0.27997,   -0.0491369,
        0.923914,     0.364806,    -0.115324,   0.00836689,
        -0.000594751,     0.302791,     0.953057,     0.135499,
                   0,            0,            0,            1;

  RigidBodyFrame<double> camera_frame(
      "Camera", tree.FindBody("iiwa_link_7"), X_7C);

  const Eigen::Isometry3d X_7T =
      Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.24)) *
      Eigen::AngleAxis<double>(-22. / 180. * M_PI, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d::UnitY());
  RigidBodyFrame<double> tool_frame("Tool",
                                    tree.FindBody("iiwa_link_7"), X_7T);

  robot_bridge::IiwaController robot_comm(tree, tool_frame, camera_frame);
  robot_comm.Start();

  Eigen::VectorXd q = Eigen::VectorXd::Zero(7);
  q << 5, 34, -4, -98, 3, 46, 22;

  robot_comm.OpenGripper();
  robot_comm.MoveJointDegrees(q, 2, true);

  Eigen::Isometry3d X0 = robot_comm.GetToolPose();

  ///////////////////////////////////////////////////////////////
  // Acquire target.
  auto real_sense = std::make_unique<rgbd_bridge::RealSenseSR300>(0);
  real_sense->set_mode(rs_ivcam_preset::RS_IVCAM_PRESET_SHORT_RANGE);
  real_sense->Start({rgbd_bridge::ImageType::RECT_RGB},
                    rgbd_bridge::ImageType::RECT_RGB);

  std::vector<Eigen::Vector2i> points = Get2Points(real_sense.get());
  for (const auto& p : points) {
    std::cout << p.transpose() << "\n";
  }

  Eigen::Isometry3d X_WC = robot_comm.GetCameraPose();
  Eigen::Vector3d pt0 = to3d(*real_sense, rgbd_bridge::ImageType::RECT_RGB, X_WC, points[0]);
  Eigen::Vector3d pt1 = to3d(*real_sense, rgbd_bridge::ImageType::RECT_RGB, X_WC, points[1]);

  std::cout << pt0.transpose() << "\n";
  std::cout << pt1.transpose() << "\n";
  Eigen::Vector3d dir = pt1 - pt0;
  double yaw = std::atan2(dir[1], dir[0]);
  if (std::fabs(yaw + M_PI) < std::fabs(yaw))
    yaw += M_PI;
  else if (std::fabs(yaw - M_PI) < std::fabs(yaw))
    yaw -= M_PI;

  std::cout << "yaw: " << yaw << "\n";

  X0.translation().head<2>() = ((pt0 + pt1) / 2).head<2>();
  X0.translation()[2] = 0.1;
  X0.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  robot_comm.MoveTool(X0, 2, true);

  ///////////////////////////////////////////////////////////////

  robot_comm.MoveStraightUntilTouch(
      Eigen::Vector3d::UnitZ(), -0.1,
      Eigen::Vector3d(100, 100, 20),
      Eigen::Vector3d::Constant(-100),
      true);

  robot_comm.CloseGripper();

  robot_comm.MoveTool(X0, 1, true);

  robot_comm.Stop();
  real_sense->Stop();

  return 0;
}
