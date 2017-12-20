#include "grasp_gen/anti_podal_grasp.h"
#include "perception/perception.h"

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cout << "need to in put cloud\n";
    exit(-1);
  }

  AntiPodalGraspPlanner grasp_planner(GRASP_PARAM_PATH);

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
  pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(argv[1], *cloud);

  cloud = perception::DownSample<pcl::PointXYZRGBNormal>(cloud, 0.004),

  grasp_planner.SetInputCloud(cloud);
  auto all_grasps =
    grasp_planner.GenerateAntipodalGrasp();


  while(true)
    ;

  return 0;
}
