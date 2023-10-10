//pcd_to_rviz.cpp
#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>
//which contains the required definitions to load and store point clouds to PCD and other file formats.
 
#include<iostream>
#include<pcl/visualization/cloud_viewer.h>


void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	//设置背景颜色
	viewer.setBackgroundColor(0, 0, 0);
}


main (int argc, char **argv)
{
  ros::init (argc, argv, "UandBdetect");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output1", 1);
  pcl::PointCloud<pcl::PointXYZI> cloud;
  sensor_msgs::PointCloud2 output;
  pcl::io::loadPCDFile ("/home/gray/Downloads/xiamen_port_map_part.pcd", cloud);//更换为自己的pcd文件路径
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcd(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile("/home/gray/Downloads/xiamen_port_map_part.pcd", *cloud_pcd) == -1) {
		return -1;
	}
  //Convert the cloud to ROS message
  pcl::toROSMsg(cloud, output);
  output.header.frame_id = "odom1";//this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer，这里时fix frame的名字
  pcl_pub.publish(output);
  	//创建viewer对象
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(cloud_pcd);
	// viewer.runOnVisualizationThreadOnce(viewerOneOff);

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
