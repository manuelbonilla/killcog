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
#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>


template<typename PointT> class Killcog : public pcl::PCLBase<PointT>
{
private:
	using pcl::PCLBase<PointT>::input_;
	typename pcl::PointCloud<PointT>::Ptr filtered_cloud_;
	typename pcl::PointCloud<PointT>::Ptr final_cloud_;
	typename pcl::PointCloud<PointT>::Ptr contour_cloud_;
	test_pcl::configConfig params_;
	dynamic_reconfigure::Server<test_pcl::configConfig> server_;
	dynamic_reconfigure::Server<test_pcl::configConfig>::CallbackType f_;
	int n_desired_points_;
	double perimeter_;
	std::vector<double > vec_perimeter_;
	std::vector<PointT> probe_points_;
	std::vector<PointT> interest_points_;
	std::vector<PointT> final_points_;
	std::vector<Eigen::Affine3f > probe_points_trans_;
	
public:
	std::vector<typename pcl::PointCloud<PointT>::Ptr> tmp_cloud_;

	bool has_results;
	Killcog();
	~Killcog() {};
	void cbParameters(test_pcl::configConfig &config, uint32_t level);
	// typename pcl::PointCloud<PointT>::Ptr compute();
	typename pcl::PointCloud<PointT>::Ptr filterCloud(const typename pcl::PointCloud<PointT>::ConstPtr &cloud, test_pcl::configConfig &config);
	void filterCloud();
	typename pcl::PointCloud<PointT>::Ptr getFilteredCloud() {return this->filtered_cloud_;};
	typename pcl::PointCloud<PointT>::Ptr getContourCloud() {return this->contour_cloud_;};
	void printParameters(test_pcl::configConfig &config);
	void setInputCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud);
	void interpolate (int n_desired_points);
	void interpolate () {this->interpolate(this->n_desired_points_);};
	void setNDesiredPoints(int n_desired_points_in) {this->n_desired_points_ = n_desired_points_in;};
	void computePerimeter(const typename pcl::PointCloud<PointT>::ConstPtr & cloud);
	std::vector<PointT> getProbePoints() {return this->probe_points_;};
	std::vector<PointT> getFinalPoints() {return this->final_points_;};
	void generateProbePointsCSYS();
	std::vector<Eigen::Affine3f> getProbePointsCSYS() {return this->probe_points_trans_;};
	void getInterestPoints();
	bool compute();


};

template<typename PointT>
Killcog<PointT>::Killcog()
{
	//original_cloud_ = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	filtered_cloud_ = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	final_cloud_ = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	contour_cloud_ = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	f_ = boost::bind(&Killcog<pcl::PointXYZ>::cbParameters, this, _1, _2);
	server_.setCallback(f_);
	this->n_desired_points_ = 0;
	this->perimeter_ = 0;
	this->has_results = false;
	// server_.getConfigDefault(this->params_);
	// this->printParameters();
	// this->filterCloud();

}

template<typename PointT>
void Killcog<PointT>::setInputCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud)
{
	this->input_ = cloud;
	this->filterCloud();
}


template<typename PointT>
void Killcog<PointT>::cbParameters(test_pcl::configConfig &config, uint32_t level) {
	/*  ROS_INFO("Reconfigure Request: %d %f %s %s %d",
	            config.int_param, config.double_param,
	            config.str_param.c_str(),
	            config.bool_param?"True":"False",
	            config.size);*/
	this->params_ = config;
	if (this->input_ && this->n_desired_points_ > 0)
	{
		this->filterCloud();
		// this->interpolate(this->n_desired_points_);
		//std::cout << "Doing something" << std::endl;
		return;
	}
	std::cout << "Doing nothing" << std::endl;

}

template<typename PointT>
void Killcog<PointT>::filterCloud()
{

	typename pcl::PointCloud<PointT>::Ptr cloud_projected (new typename pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr cloud_filtered_xyz (new typename pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new typename pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr cloud_hull (new typename pcl::PointCloud<PointT>);

	cloud_filtered_xyz = this->filterCloud(this->input_, this->params_);

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

// pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	// coefficients->values.resize (4);
	// coefficients->values[0] = coefficients->values[1] = 0;
	// coefficients->values[2] = 1.0;
	// coefficients->values[3] = coefficients;

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

	this->filtered_cloud_ = cloud_filtered;
	this->contour_cloud_ = cloud_hull;
}


template<typename PointT>
void Killcog<PointT>::interpolate(int n_desired_points_in)
{

	if (!(this->input_ && this->n_desired_points_ > 0))
	{
		std::cout << "Either cloud or num interpolated points undefined" << std::endl;
		return;
	}
	typename pcl::PointCloud<PointT>::Ptr local_cloud (new typename pcl::PointCloud<PointT>);
	local_cloud = this->contour_cloud_;
	computePerimeter(local_cloud);

	double calc_step_size = this->perimeter_ / n_desired_points_in;

	int j = 1;
	int index_toe = 0;

	std::vector<PointT> tmp_points;
	tmp_points.push_back(local_cloud->points[0]);

	for (int i = 1; i < local_cloud->width; i++)
	{
		if  ( this->vec_perimeter_[i] >= calc_step_size * j)
		{
			PointT p1 = local_cloud->points[i - 1];
			PointT p2 = local_cloud->points[i];
			double t = ((calc_step_size * j) - (this->vec_perimeter_[i - 1])) / ( this->vec_perimeter_[i] - this->vec_perimeter_[i - 1]);
			tmp_points.push_back( PointT((1.0 - t) * p1.x + t * p2.x, (1.0 - t) * p1.y + t * p2.y, local_cloud->points[0].z));
			if (tmp_points.size() > 1)
			{
				if (tmp_points[tmp_points.size() - 1].x > tmp_points[index_toe].x)
				{
					index_toe = tmp_points.size() - 1 ;
				}
			}
			i--;
			j++;
		}
	}
	std::vector<PointT> tmp_points_sorted;
	for (int i = index_toe; i < tmp_points.size(); i++)
		tmp_points_sorted.push_back(tmp_points[i]);
	for (int i = 0; i < index_toe; i++)
		tmp_points_sorted.push_back(tmp_points[i]);

	this->probe_points_ = tmp_points_sorted;

}


template<typename PointT>
void  Killcog<PointT>::computePerimeter(const typename pcl::PointCloud<PointT>::ConstPtr &cloud)
{

	double p = 0;
	int n_points = cloud->width;
	this->vec_perimeter_ = std::vector<double>(n_points, 0);
	int i;
	for (i = 0; i < n_points - 1; i++)
	{
		this->vec_perimeter_[i] = p;
		p = p + (cloud->points[i + 1].getVector3fMap() - cloud->points[i].getVector3fMap()).norm();
	}
	this->vec_perimeter_[i] = p;
	p = p + (cloud->points[0].getVector3fMap() - cloud->points[n_points - 1].getVector3fMap()).norm();
	this->perimeter_ = p;


}

template<typename PointT>
void Killcog<PointT>::generateProbePointsCSYS ()
{
	float angle = 0;
	int n_points = this->probe_points_.size();
	this->probe_points_trans_.clear();
	Eigen::Vector3f line1 = Eigen::Vector3f(1.0, 0.0, 0.0);
	for (int i = 0; i < n_points; i++)
	{
		Eigen::Vector3f line2 = Eigen::Vector3f(1.0, 0.0, 0.0);
		Eigen::Affine3f trans;
		if (i == 0)
		{
			line2 =  this->probe_points_[1].getVector3fMap()  - this->probe_points_[n_points - 1].getVector3fMap() ;
		}
		else
		{
			if (i == n_points - 1)
			{
				line2 =  this->probe_points_[0].getVector3fMap()  - this->probe_points_[n_points - 2].getVector3fMap() ;
			}
			else
			{
				line2 =  this->probe_points_[i + 1].getVector3fMap()  - this->probe_points_[i - 1].getVector3fMap() ;
			}
		}
		line2.normalize();
		angle = std::atan2(line2(1), line2(0));
		this->probe_points_trans_.push_back(Eigen::Translation3f(this->probe_points_[i].getVector3fMap()) * Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
	}

}


template<typename PointT>
void Killcog<PointT>::getInterestPoints ()
{
	std::vector<PointT> interest_points_local;
	std::vector<typename pcl::PointCloud<PointT>::Ptr> tmp_cloud_local;
	std::vector<PointT> final_points_local;
	for (int i = 0; i < this->probe_points_.size(); i++)
		// for (int i = 0; i < 1; i++)
	{
		typename pcl::PointCloud<PointT>::Ptr local_cloud (new typename pcl::PointCloud<PointT>);
		typename pcl::PointCloud<PointT>::Ptr local_cloud_filtered (new typename pcl::PointCloud<PointT>);
		typename pcl::PointCloud<PointT>::Ptr local_cloud_filtered_projected (new typename pcl::PointCloud<PointT>);
		pcl::transformPointCloud (*this->filtered_cloud_, *local_cloud, this->probe_points_trans_[i].inverse());

		test_pcl::configConfig config_local = this->params_;
		config_local.x_min = -this->params_.length_of_tracers_x/2.0;
		config_local.x_max = this->params_.length_of_tracers_x/2.0;
		config_local.y_min = -this->params_.width_of_tracers_y/2.0;
		config_local.y_max = this->params_.width_of_tracers_y/2.0;
		config_local.z_min = -this->params_.height_of_tracers_z/2.0;
		config_local.z_max = this->params_.height_of_tracers_z/2.0;
		// std::cout << i  << " ";
		local_cloud_filtered = this->filterCloud(local_cloud, config_local);


		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		coefficients->values.resize (4);
		coefficients->values[1] = coefficients->values[2] = coefficients->values[3] = 0 ;
		coefficients->values[0] = 1.0;
		pcl::ProjectInliers<PointT> proj;
		proj.setModelType (pcl::SACMODEL_PLANE);
		//proj.setIndices (inliers);
		proj.setInputCloud (local_cloud_filtered);
		proj.setModelCoefficients (coefficients);
		proj.filter (*local_cloud_filtered_projected);


		auto z_max = std::max_element(local_cloud_filtered_projected->begin(), local_cloud_filtered_projected->end(),
            [] (PointT const  a, PointT const  b) {
            return a.z > b.z; });
		Eigen::Vector3f p_local = this->probe_points_trans_[i]*z_max->getVector3fMap();
		final_points_local.push_back(PointT(p_local[0],p_local[1],p_local[2]));

		typename pcl::PointCloud<PointT>::Ptr local_cloud_proj (new typename pcl::PointCloud<PointT>);
		pcl::transformPointCloud (*local_cloud_filtered_projected, *local_cloud_proj, this->probe_points_trans_[i]);
		tmp_cloud_local.push_back(local_cloud_proj);
	}


	this->interest_points_ = interest_points_local;
	this->tmp_cloud_ = tmp_cloud_local;
	this->final_points_ = final_points_local;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr Killcog<PointT>::filterCloud(const typename pcl::PointCloud<PointT>::ConstPtr &cloud, test_pcl::configConfig &config)
{
	typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new typename pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr cloud_filtered_x (new typename pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr cloud_filtered_xy (new typename pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr cloud_filtered_xyz (new typename pcl::PointCloud<PointT>);

	// this->printParameters(config);
	pcl::PassThrough<PointT> pass_x;
	pass_x.setInputCloud (cloud);
	pass_x.setFilterFieldName ("x");
	pass_x.setFilterLimits (config.x_min, config.x_max);
	//pass_x.setFilterLimitsNegative (true);
	pass_x.filter (*cloud_filtered_x);

	// Create the filtering object y
	pcl::PassThrough<PointT> pass_y;
	pass_y.setInputCloud (cloud_filtered_x);
	pass_y.setFilterFieldName ("y");
	pass_y.setFilterLimits (config.y_min, config.y_max);
	//pass_y.setFilterLimitsNegative (true);
	pass_y.filter (*cloud_filtered_xy);

	// Create the filtering object z
	pcl::PassThrough<PointT> pass_z;
	pass_z.setInputCloud (cloud_filtered_xy);
	pass_z.setFilterFieldName ("z");
	pass_z.setFilterLimits (config.z_min, config.z_max);
	//pass_z.setFilterLimitsNegative (true);
	pass_z.filter (*cloud_filtered_xyz);


	return cloud_filtered_xyz;
}

template<typename PointT>
bool Killcog<PointT>::compute()
{

	this->interpolate();
    this->generateProbePointsCSYS();
    this->getInterestPoints();
    this->has_results = true;

    return true;
}

template<typename PointT>
void Killcog<PointT>::printParameters(test_pcl::configConfig &config)
{
	std::cout << "params: " << std::endl;
	std::cout << "x_min: " << config.x_min << std::endl;
	std::cout << "x_max: " << config.x_max << std::endl;
	std::cout << "y_min: " << config.y_min << std::endl;
	std::cout << "y_max: " << config.y_max << std::endl;
	std::cout << "z_min: " << config.z_min << std::endl;
	std::cout << "z_max: " << config.z_max << std::endl;
	std::cout << "distance_plane_threshold: " << config.distance_plane_threshold << std::endl;
	std::cout << "convex_hull_apha: " << config.convex_hull_apha << std::endl;
	std::cout << "length_of_tracers_x: " << config.length_of_tracers_x << std::endl;
	std::cout << "width_of_tracers_y: " << config.width_of_tracers_y << std::endl;
	std::cout << "height_of_tracers_z: " << config.height_of_tracers_z << std::endl;

}


#endif