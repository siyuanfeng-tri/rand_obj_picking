#include "util.h"

#include <fstream>

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

double FitObj(
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr &obj,
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr &scene,
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &aligned_obj,
    Eigen::Isometry3f *obj_pose,
    lcm::LCM* lcm) {

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tmp =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();

  // Initial guess.
  pcl::transformPointCloudWithNormals(*obj, *aligned_obj, *obj_pose);

  double score;
  Eigen::Isometry3f incremental;

  for (int it = 0; ; it++) {
    score = perception::AlignCloud<pcl::PointXYZRGBNormal>(
        aligned_obj, scene, 0.1, tmp, &incremental);
    *obj_pose = incremental * (*obj_pose);
    if (score < 0)
      return score;

    aligned_obj.swap(tmp);

    if (lcm) {
      perception::VisualizePointCloudDrake(*aligned_obj, lcm,
          Eigen::Isometry3d::Identity(),
          "DRAKE_POINTCLOUD_fitted");
    }

    if (IsIdentity<float>(incremental, 0.001, 0.0001))
      break;
  }

  return score;
}


