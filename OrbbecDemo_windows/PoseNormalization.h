#pragma once
#include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/surface/on_nurbs/fitting_curve_pdm.h>
#include <pcl/surface/3rdparty/opennurbs/opennurbs_curve.h>
#include <pcl/surface/3rdparty/opennurbs/opennurbs_point.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/common/distances.h>
#include <pcl/common/common_headers.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
class PoseNormalization
{
public:
	PoseNormalization();
	/*!
	* reset the global paramters
	*/
	void Reset();
	/*!
	* set the cloudfile of object with pcd format
	*/
	void SetInputcloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input);
	/*!
	* set ground plane,direction of normal must face up.
	*/
	void Set_gd_plane(Eigen::VectorXf coefficients_gdplane);
	/*!
	* set symmetry plane
	*/
	void Set_symmetry_plane(pcl::ModelCoefficients coefficients_symmetry_plane);
	/*!
	* set head direction
	*/
	void Set_head_direction(Eigen::Vector3f head_direction);
	void Computing_thread();
	void Construct_transformation();
	void Do_transformation();
public:
	void Get_gd_plane(Eigen::VectorXf& coefficients_gdplane);
	void Get_symmetry_plane(pcl::ModelCoefficients& coefficients_symmetry_plane);
	pcl::PointCloud<pcl::PointXYZRGBA> Get_normalizated_data();
private:
	Eigen::Vector3f X_vector, Y_vector, Z_vector;
	Eigen::Vector3f O_center;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input_;/*!< cloud_input store the pointer of input cloud object*/
	pcl::PointCloud<pcl::PointXYZRGBA> cloud_output_;
	bool gdplane_ok_, symmetry_planeok_, input_cloud_ok_, head_d_ok_;/**< input will be set to true if the client SetInputcloud*/
	Eigen::Vector3f head_direction_;
	Eigen::VectorXf coefficients_gdplane_, coefficients_gdplane_t_;
	pcl::ModelCoefficients coefficients_symmetry_plane_, coefficients_symmetry_plane_t_;
	Eigen::Matrix4f T_SVD;

};
