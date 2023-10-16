#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <glog/logging.h>
#include "Eigen/Core"
#include "Eigen/Geometry"


DEFINE_string(path, "/media/gray/File/东风算法赛_数据集/岸桥场景/斜行变道", "PCD文件路径");
DEFINE_string(pcd_path, FLAGS_path + "/map.pcd", "PCD文件路径");
DEFINE_string(pose_path, FLAGS_path +"/first_pose.txt", "初始位姿文件路径");

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	// 设置背景颜色
	viewer.setBackgroundColor(0, 0, 0);
}

main (int argc, char **argv)
{
  // glog
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  // ros
  ros::init (argc, argv, "SaveAddPCD");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1);

  // 导入pcd文件
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcd(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile(FLAGS_pcd_path, *cloud_pcd) == -1) {
    return -1;
  }

  // 读取初始位姿
  std::ifstream pose_file(FLAGS_pose_path);
  std::string pose_line;
  std::getline(pose_file, pose_line);
  std::istringstream pose_stream(pose_line);
  double pose_data[7];
  for (int i = 0; i < 7; ++i) {
    pose_stream >> pose_data[i];
  }
  Eigen::Matrix3d rot;
  rot = Eigen::AngleAxisd(pose_data[3], Eigen::Vector3d::UnitZ())
       *Eigen::AngleAxisd(pose_data[1], Eigen::Vector3d::UnitY())
       *Eigen::AngleAxisd(pose_data[2], Eigen::Vector3d::UnitX());
  Eigen::Vector3d trans{pose_data[4],pose_data[5],pose_data[6]};
  Eigen::Affine3d transform;
  transform.linear() = rot;
  transform.translation() = trans;
  
  // 点云变换
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcd_transformed(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloud_pcd,*cloud_pcd_transformed,transform);

  //创建viewer对象
  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  viewer.showCloud(cloud_pcd_transformed);

  
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
