#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "lcm/lcm-cpp.hpp"

template <typename T>
bool IsIdentity(const Eigen::Transform<T, 3, Eigen::Isometry> &transform,
                T ang_thresh, T pos_thresh) {
  auto ang_err = Eigen::AngleAxis<T>(transform.linear());
  auto pos_err = transform.translation();

  return (std::fabs(ang_err.angle()) < ang_thresh &&
          pos_err.norm() < pos_thresh);
}

std::vector<Eigen::VectorXd> read_q(
    const std::string& file_name, int line_size);

void FitObj(
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr &obj,
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr &scene,
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &aligned_obj,
    Eigen::Isometry3f *obj_pose,
    double *score,
    lcm::LCM* lcm);

double ThreadedFitObj(
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr &obj,
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr &scene,
    const std::vector<Eigen::Isometry3f>& obj_initial_guess,
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &aligned_obj,
    Eigen::Isometry3f *obj_pose,
    lcm::LCM* lcm);
