// PCL specific includes
#ifndef TEST_PCL_H
#define TEST_PCL_H

#include <iostream>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>

#include <dynamic_reconfigure/server.h>
#include <test_pcl/configConfig.h>

#include <pcl/pcl_base.h>
#include <pcl/common/io.h>
#include <pcl/conversions.h>
#include <pcl/filters/boost.h>
#include <cfloat>
#include <pcl/PointIndices.h>

template<typename PointT> class Killcog : public pcl::PCLBase<PointT>
{
private:
  using pcl::PCLBase<PointT>::input_;
  typename pcl::PointCloud<PointT>::Ptr filtered_cloud_;
  typename pcl::PointCloud<PointT>::Ptr final_cloud_;
  test_pcl::configConfig params_;
  dynamic_reconfigure::Server<test_pcl::configConfig> server_;
  dynamic_reconfigure::Server<test_pcl::configConfig>::CallbackType f_;

public:
  std::vector<typename pcl::PointCloud<PointT>::Ptr> tmp_cloud_;


  Killcog();
  ~Killcog(){};
  void cbParameters(test_pcl::configConfig &config, uint32_t level);
  typename pcl::PointCloud<PointT>::Ptr compute();
  void filterCloud();
  typename pcl::PointCloud<PointT>::Ptr getFilteredCloud() {return this->filtered_cloud_;};
  void printParameters();
  void setInputCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud);
  //void setInputCloud(const typename pcl::PointCloud<PointT>::ConstPtr &in_cloud) {this->original_cloud_ = in_cloud;};


};

template<typename PointT>
Killcog<PointT>::Killcog()
{
	//original_cloud_ = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	filtered_cloud_ = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	final_cloud_ = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	f_ = boost::bind(&Killcog<pcl::PointXYZ>::cbParameters, this, _1, _2);
  	server_.setCallback(f_);
  	// server_.getConfigDefault(this->params_);
  	// this->printParameters();
  	// this->filterCloud();

}

template<typename PointT>
void Killcog<PointT>::setInputCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud)
{
	this->input_ = cloud;
	this->filterCloud();
};


template<typename PointT>
void Killcog<PointT>::cbParameters(test_pcl::configConfig &config, uint32_t level) {
  /*  ROS_INFO("Reconfigure Request: %d %f %s %s %d",
              config.int_param, config.double_param,
              config.str_param.c_str(),
              config.bool_param?"True":"False",
              config.size);*/
  this->params_ = config;
  if (this->input_)
  	this->filterCloud();

}

template<typename PointT>
void Killcog<PointT>::filterCloud()
{

  typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new typename pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr cloud_filtered_x (new typename pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr cloud_filtered_xy (new typename pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr cloud_filtered_xyz (new typename pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr cloud_projected (new typename pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr cloud_hull (new typename pcl::PointCloud<PointT>);

  pcl::PassThrough<PointT> pass_x;
  pass_x.setInputCloud (this->input_);
  pass_x.setFilterFieldName ("x");
  pass_x.setFilterLimits (this->params_.x_min, this->params_.x_max);
  //pass_x.setFilterLimitsNegative (true);
  pass_x.filter (*cloud_filtered_x);

  //I'm not sure if I need to create new objects for each passthrough or if I can just reset it each time with new settings.
  //I'm going to be safe and create new objects for each.

  // Create the filtering object y
  pcl::PassThrough<PointT> pass_y;
  pass_y.setInputCloud (cloud_filtered_x);
  pass_y.setFilterFieldName ("y");
  pass_y.setFilterLimits (this->params_.y_min, this->params_.y_max);
  //pass_y.setFilterLimitsNegative (true);
  pass_y.filter (*cloud_filtered_xy);

  // Create the filtering object z
  pcl::PassThrough<PointT> pass_z;
  pass_z.setInputCloud (cloud_filtered_xy);
  pass_z.setFilterFieldName ("z");
  pass_z.setFilterLimits (this->params_.z_min, this->params_.z_max);
  //pass_z.setFilterLimitsNegative (true);
  pass_z.filter (*cloud_filtered_xyz);


  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (this->params_.distance_plane_threshold);

  seg.setInputCloud (cloud_filtered_xyz);
  seg.segment (*inliers, *coefficients);
  pcl::ExtractIndices<PointT> extract;

  extract.setInputCloud (cloud_filtered_xyz);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*cloud_filtered);


  pcl::ProjectInliers<PointT> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  //proj.setIndices (inliers);
  proj.setInputCloud (cloud_filtered);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);

  pcl::ConcaveHull<PointT> chull;
  chull.setInputCloud (cloud_projected);
  chull.setAlpha (this->params_.convex_hull_apha);
  chull.reconstruct (*cloud_hull);

  this->filtered_cloud_ = cloud_hull;
  // this->filtered_cloud_ = cloud_filtered_xyz;
}


template<typename PointT>
void Killcog<PointT>::printParameters()
{
	std::cout << "params: " << std::endl;
  	std::cout << "x_min: " << this->params_.x_min << std::endl;
  	std::cout << "x_min: " << this->params_.x_max << std::endl;
  	std::cout << "y_min: " << this->params_.y_min << std::endl;
  	std::cout << "y_min: " << this->params_.y_max << std::endl;
  	std::cout << "z_min: " << this->params_.z_min << std::endl;
  	std::cout << "z_min: " << this->params_.z_max << std::endl;
  	std::cout << "distance_plane_threshold: " << this->params_.distance_plane_threshold << std::endl;
  	std::cout << "convex_hull_apha: " << this->params_.convex_hull_apha << std::endl;
}


#endif