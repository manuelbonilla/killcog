#include <test_pcl/test_pcl.h>

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("contour", 1);
  ros::Publisher pub_ori = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("original_cloud", 1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xy (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PLYReader Reader;
  Reader.read("test1.ply", *cloud);

  //PointCloud::Ptr msg (new PointCloud);
  cloud->header.frame_id = "map";
  //msg->height = msg->width = 1;
  //msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));
  //ROS_INFO_STREAM("Num Points = " << cloud->points.size());

  // Create the filtering object x
  pcl::PassThrough<pcl::PointXYZ> pass_x;
  pass_x.setInputCloud (cloud);
  pass_x.setFilterFieldName ("x");
  pass_x.setFilterLimits (-.16, .12);
  //pass_x.setFilterLimitsNegative (true);
  pass_x.filter (*cloud_filtered_x);

  //I'm not sure if I need to create new objects for each passthrough or if I can just reset it each time with new settings.
  //I'm going to be safe and create new objects for each.

  // Create the filtering object y
  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setInputCloud (cloud_filtered_x);
  pass_y.setFilterFieldName ("y");
  pass_y.setFilterLimits (-.1, .1);
  //pass_y.setFilterLimitsNegative (true);
  pass_y.filter (*cloud_filtered_xy);

  // Create the filtering object z
  pcl::PassThrough<pcl::PointXYZ> pass_z;
  pass_z.setInputCloud (cloud_filtered_xy);
  pass_z.setFilterFieldName ("z");
  pass_z.setFilterLimits (0.3, .5);
  //pass_z.setFilterLimitsNegative (true);
  pass_z.filter (*cloud_filtered_xyz);


  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.003);

  seg.setInputCloud (cloud_filtered_xyz);
  seg.segment (*inliers, *coefficients);
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  extract.setInputCloud (cloud_filtered_xyz);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*cloud_filtered);


  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  //proj.setIndices (inliers);
  proj.setInputCloud (cloud_filtered);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);

  pcl::ConcaveHull<pcl::PointXYZ> chull;
  chull.setInputCloud (cloud_projected);
  chull.setAlpha (0.01);
  chull.reconstruct (*cloud_hull);

  ros::Rate loop_rate(100);


  while (nh.ok())
  {
    pcl_conversions::toPCL(ros::Time::now(), cloud_filtered_xyz->header.stamp);
    pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
    //cloud->header.stamp = ros::Time::now().toNSec();
    pub.publish (cloud_hull);
    pub_ori.publish (cloud);
    ros::spinOnce ();
    loop_rate.sleep ();
  }

}
