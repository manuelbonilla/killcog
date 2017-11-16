
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <test_pcl/test_pcl.h>
#include <pcl_ros/point_cloud.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <rviz_visual_tools/tf_visual_tools.h>


int
main (int argc, char** argv)
{
  ros::init (argc, argv, "test_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub_contour = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("contour", 1);
  ros::Publisher pub_filtered = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("filtered", 1);
  ros::Publisher pub_ori = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("original_cloud", 1);
  ros::Publisher pub_slice = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("cloud_slice", 1);
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world", "/rviz_visual_markers"));
  rviz_visual_tools::TFVisualTools tf_visualizer;




  Killcog<pcl::PointXYZ> *killcog = new Killcog<pcl::PointXYZ>();

  // ros::ServiceServer srv_server = nh.advertiseService("compute", &Killcog<pcl::PointXYZ>::compute, (Killcog<pcl::PointXYZ> *) killcog);

// ("process_voice_commands", boost::bind(processVoiceCommands, _1, _2, &sensor_message_gateway_open_client))

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr contour_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PLYReader Reader;
  std::string file_name;
  nh.param<std::string>("file_name", file_name, "test2.ply");
  ROS_INFO_STREAM("Working with Cloud: " << file_name);
  Reader.read(file_name, *cloud);
  cloud->header.frame_id = "map";
  killcog->setInputCloud(cloud);
  killcog->setNDesiredPoints(30);

  ros::Rate loop_rate(10);

  while (nh.ok())
  {
    //killcog->filterCloud();
    contour_cloud = killcog->getContourCloud();
    final_cloud = killcog->getFilteredCloud();
    pcl_conversions::toPCL(ros::Time::now(), final_cloud->header.stamp);
    pcl_conversions::toPCL(ros::Time::now(), contour_cloud->header.stamp);
    pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
    pub_contour.publish (contour_cloud);
    pub_filtered.publish (final_cloud);
    pub_ori.publish (cloud);

    killcog->compute();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> slices = killcog->tmp_cloud_;
    pcl::PointCloud<pcl::PointXYZ> p1 = *slices[0];

    for (int i = 1; i < slices.size(); ++i)
    {
      p1 += *slices[i];
      // pcl::PointCloud<pcl::PointXYZ>::Ptr p2= slices[i];
      // pcl::concatenateFields ( p1, p2, all_slices);

    }

    pub_slice.publish(p1);
    
    std::vector<pcl::PointXYZ> probe_points = killcog->getProbePoints();
    std::vector<Eigen::Affine3f> probe_points_CSYS = killcog->getProbePointsCSYS();
    std::vector<pcl::PointXYZ> final_points = killcog->getFinalPoints();

    visual_tools_->deleteAllMarkers();
    for (int i = 0; i < probe_points.size(); i++)
    {
      Eigen::Vector3d point(probe_points[i].x, probe_points[i].y, probe_points[i].z);
      visual_tools_->publishSphere(point, rviz_visual_tools::BLUE, rviz_visual_tools::XXSMALL, std::string("probe_point"));
      Eigen::Vector3d point_f(final_points[i].x, final_points[i].y, final_points[i].z);
      visual_tools_->publishSphere(point_f, rviz_visual_tools::RED, rviz_visual_tools::XXSMALL, std::string("point"));
      tf_visualizer.publishTransform(probe_points_CSYS[i].cast<double>(), "world", std::string("probe_point") + std::to_string(i));
    }
    visual_tools_->trigger();
    // }


    ros::spinOnce ();
    loop_rate.sleep ();
  }

}