#include "grasp_gen/anti_podal_grasp.h"
#include "perception/perception.h"

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cout << "need to in put cloud\n";
    exit(-1);
  }

  AntiPodalGraspPlanner grasp_planner(GRASP_PARAM_PATH);

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr centered_cloud =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();

  pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(argv[1], *cloud);
  Eigen::Isometry3f tf(Eigen::Translation3f(Eigen::Vector3f(
          std::atof(argv[2]), std::atof(argv[3]), 0)));

  pcl::transformPointCloudWithNormals(*cloud, *centered_cloud, tf);
  // perception::VisualizePointCloudAndNormal(centered_cloud);

  // pcl::io::savePCDFileASCII(std::string(argv[1]), *centered_cloud);

  cloud = perception::DownSample<pcl::PointXYZRGBNormal>(cloud, 0.004);
  Eigen::Vector4f mid_pt;
  pcl::compute3DCentroid(*cloud, mid_pt);
  std::cout << "central: " << mid_pt.transpose() << "\n";

  grasp_planner.SetInputCloud(cloud);
  auto all_grasps = grasp_planner.GenerateAntipodalGrasp();

  while(true)
    ;

  return 0;
}
