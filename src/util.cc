#include "util.h"

#include <fstream>
#include <thread>

#include "perception/perception.h"

std::vector<Eigen::VectorXd> read_q(
    const std::string& file_name, int line_size) {
  std::vector<Eigen::VectorXd> setpoints;

  std::ifstream ifs;
  ifs.open(file_name, std::ifstream::in);
  if (!ifs.is_open())
    return setpoints;

  while(ifs.peek() != EOF) {
    Eigen::VectorXd q(line_size);
    for (int i = 0; i < line_size; i++) {
      ifs >> q(i);
      if (ifs.eof() || ifs.fail()) {
        return setpoints;
      }
    }
    setpoints.push_back(q);
  }

  return setpoints;
}

void FitObj(
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr &obj,
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr &scene,
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &aligned_obj,
    Eigen::Isometry3f *obj_pose,
    double *score,
    lcm::LCM* lcm) {

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tmp =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();

  // Initial guess.
  pcl::transformPointCloudWithNormals(*obj, *aligned_obj, *obj_pose);

  Eigen::Isometry3f incremental;

  for (int it = 0; ; it++) {
    *score = perception::AlignCloud<pcl::PointXYZRGBNormal>(
        aligned_obj, scene, 0.05, tmp, &incremental);
    *obj_pose = incremental * (*obj_pose);
    if (*score < 0)
      return;

    aligned_obj.swap(tmp);

    if (lcm) {
      perception::VisualizePointCloudDrake(*aligned_obj, lcm,
          Eigen::Isometry3d::Identity(),
          "DRAKE_POINTCLOUD_fitted");
    }

    if (IsIdentity<float>(incremental, 0.001, 0.0001))
      break;
  }
}

double ThreadedFitObj(
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr &obj,
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr &scene,
    const std::vector<Eigen::Isometry3f>& obj_initial_guess,
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &aligned_obj,
    Eigen::Isometry3f *obj_pose,
    lcm::LCM* lcm) {
  const size_t num = obj_initial_guess.size();
  std::vector<double> scores(num);
  std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> results(num);
  std::vector<Eigen::Isometry3f> poses(num);
  std::vector<std::thread> threads;

  for (size_t i = 0; i < num; i++) {
    results[i] = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
    poses[i] = obj_initial_guess[i];

    threads.push_back(std::thread(std::bind(FitObj, obj, scene, results[i], &poses[i], &scores[i], nullptr)));
  }

  double best_score = 1e5;
  size_t best_idx;
  for (size_t i = 0; i < num; i++) {
    threads[i].join();

    if (scores[i] < best_score) {
      best_score = scores[i];
      best_idx = i;
    }
  }

  *obj_pose = poses[best_idx];
  aligned_obj = results[best_idx];

  if (lcm) {
      perception::VisualizePointCloudDrake(*aligned_obj, lcm,
          Eigen::Isometry3d::Identity(),
          "DRAKE_POINTCLOUD_fitted");
  }

  return best_score;
}
