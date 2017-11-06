#include <ros/ros.h>
#include <test_pcl/test_pcl.h>
#include <pcl_ros/point_cloud.h>


int
main (int argc, char** argv)
{
  ros::init (argc, argv, "test_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("contour", 1);
  ros::Publisher pub_ori = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("original_cloud", 1);
  

  Killcog<pcl::PointXYZ> *killcog = new Killcog<pcl::PointXYZ>();


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PLYReader Reader;
  Reader.read("test1.ply", *cloud);
  cloud->header.frame_id = "map";
  killcog->setInputCloud(cloud);

  ros::Rate loop_rate(1000);

  while (nh.ok())
  {
    // killcog->filterCloud();
    final_cloud = killcog->getFilteredCloud();
    pcl_conversions::toPCL(ros::Time::now(), final_cloud->header.stamp);
    pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
    pub.publish (final_cloud);
    pub_ori.publish (cloud);
    ros::spinOnce ();
    loop_rate.sleep ();
  }

}