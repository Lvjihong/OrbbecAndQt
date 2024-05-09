#pragma once
#include <pcl/visualization/pcl_visualizer.h>
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
class SymmetricalPlane
{
public:
	SymmetricalPlane();
public:
	void Reset();
	void SetBiggestProfile(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr profile);
	void SetUpVector(Eigen::VectorXf up_normal_plane);
	void SetSearchRadius(float radius);
	pcl::ModelCoefficients get_estimated_plane();
	pcl::PointXYZRGBA get_point_onplane();
	void Do_Computing_thread();
	void Computing_profile_longest_direction();
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Search_current_samples_foriteration(float search_radius, pcl::PointXYZRGBA current_point);
	pcl::PointXYZRGBA MirrorReflection_onePoint_along_plane(pcl::PointXYZRGBA sample_point, pcl::PointXYZRGBA plane_point, pcl::Normal plane_normal);
	void PointToPlaneDistance(pcl::PointXYZRGBA sample_point, pcl::PointXYZRGBA plane_point, pcl::Normal plane_normal, float& distance_return, Eigen::Vector3f& pedal_return);
	int Ransac_Test(float search_radius, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_profiles, pcl::PointXYZRGBA plane_point, pcl::Normal plane_normal);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Intersection_plane_with_object_computing(Eigen::Vector4f plane, float threshold, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr profile);
	Eigen::Vector4f Constructing_vplane_along_vector_moved(Eigen::Vector4f vector, Eigen::Vector4f plane_old, float distance_moved);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getColoredCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_, float r, float g, float b);


private:
	//ransac based method related
	bool biggest_profile_input_, up_vector_input_;
	float search_radius_;//radius determine the size of sample getted in Search_current_samples_foriteration()
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr biggest_profile_;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr biggest_profile_1;
	pcl::PointXYZRGBA result_point_onplane_;  //one point for final symmetry plane
	pcl::Normal result_normal_forplane_; //normal for final symmetry plane
	Eigen::Vector3f PCA_X_Vector, PCA_Y_Vector, PCA_Z_Vector; // profile PCA result 
	Eigen::Vector3f biggest_profile_mass_center; // pca center
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr current_samples_for_eachiteration_;
	Eigen::VectorXf up_normal_plane_;//for store the upvector of ground plane
	pcl::Normal normal_for_contructing_plane_; //normal contructing by normal of ground plane and vector of  PCA_X_Vector
	float ratio_sample_consensus_;
	pcl::ModelCoefficients symmetry_plane_estimated_coeff;
};
