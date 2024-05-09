#include "PoseNormalization.h"

/**
	* \class PoseNormalization
	*this class was designed for pose normalzation based on information of symmetry plane, ground plane and head direction.
	*/

PoseNormalization::PoseNormalization() :gdplane_ok_(false), symmetry_planeok_(false), input_cloud_ok_(false), head_d_ok_(false)
{
	cloud_input_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
	coefficients_symmetry_plane_.values.resize(4);
	coefficients_symmetry_plane_t_.values.resize(4);
	coefficients_gdplane_t_.resize(4);
	coefficients_gdplane_.resize(4);
}
/*!
* reset the global paramters
*/
void PoseNormalization::Reset()
{
	cloud_input_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
	gdplane_ok_ = false;
	symmetry_planeok_ = false;
	input_cloud_ok_ = false;
	head_d_ok_ = false;
}
/*!
* set the cloudfile of object with pcd format
*/
void PoseNormalization::SetInputcloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input)
{
	cloud_input_ = input;
	input_cloud_ok_ = true;
}
/*!
* set ground plane
*/
void PoseNormalization::Set_gd_plane(Eigen::VectorXf coefficients_gdplane)
{

	coefficients_gdplane_ = -coefficients_gdplane;////////////////////////////这里改成Z轴向下为正
	gdplane_ok_ = true;

}
/*!
* set symmetry plane
*/
void PoseNormalization::Set_symmetry_plane(pcl::ModelCoefficients coefficients_symmetry_plane)
{
	coefficients_symmetry_plane_ = coefficients_symmetry_plane;
	symmetry_planeok_ = true;
}
/*!
* set head direction
*/
void PoseNormalization::Set_head_direction(Eigen::Vector3f head_direction)
{
	head_direction_ = head_direction;
	head_d_ok_ = true;
}

void PoseNormalization::Computing_thread()
{
	Construct_transformation();
	//cout << "wuhuuuuuuuuuuuuu" << endl;
	Do_transformation();
}
void PoseNormalization::Construct_transformation()//修改一下
{
	Z_vector = coefficients_gdplane_.head(3);//Z轴为法线方向
	Y_vector(0) = coefficients_symmetry_plane_.values.at(0);//y轴对称平面方向
	Y_vector(1) = coefficients_symmetry_plane_.values.at(1);
	Y_vector(2) = coefficients_symmetry_plane_.values.at(2);
	X_vector = Y_vector.cross(Z_vector);
	X_vector.normalize();//归一化
	Y_vector.normalize();
	Z_vector.normalize();
	if (X_vector.dot(head_direction_) <= 0)
	{
		X_vector = -X_vector;
		Y_vector = -Y_vector;
	}
	Eigen::Vector4f seg_result_centroid;
	pcl::compute3DCentroid(*cloud_input_, seg_result_centroid);//质心
	//project the centroid onto the symmetry plane
	Eigen::Vector3f normal_sysplane_v(coefficients_symmetry_plane_.values[0], coefficients_symmetry_plane_.values[1],
		coefficients_symmetry_plane_.values[2]);
	Eigen::Vector3f Point_one_sysplane_v(-coefficients_symmetry_plane_.values[0] * coefficients_symmetry_plane_.values[3],
		-coefficients_symmetry_plane_.values[1] * coefficients_symmetry_plane_.values[3],
		-coefficients_symmetry_plane_.values[2] * coefficients_symmetry_plane_.values[3]);
	float sn = -normal_sysplane_v.dot(seg_result_centroid.head(3) - Point_one_sysplane_v);
	float sd = sqrt(normal_sysplane_v.dot(normal_sysplane_v));
	float sb = sn / sd;
	seg_result_centroid.head(3) = seg_result_centroid.head(3) + sb * normal_sysplane_v;

	pcl::PointXYZ center(seg_result_centroid(0), seg_result_centroid(1), seg_result_centroid(2));
	pcl::PointXYZ x_axis(X_vector(0) + seg_result_centroid(0), X_vector(1) + seg_result_centroid(1), X_vector(2) + seg_result_centroid(2));
	pcl::PointXYZ y_axis(Y_vector(0) + seg_result_centroid(0), Y_vector(1) + seg_result_centroid(1), Y_vector(2) + seg_result_centroid(2));
	pcl::PointXYZ z_axis(Z_vector(0) + seg_result_centroid(0), Z_vector(1) + seg_result_centroid(1), Z_vector(2) + seg_result_centroid(2));

	pcl::PointXYZ x_axis_t(1, 0, 0);
	pcl::PointXYZ y_axis_t(0, 1, 0);
	pcl::PointXYZ z_axis_t(0, 0, 1);
	pcl::PointCloud<pcl::PointXYZ> cloudsource;
	pcl::PointCloud<pcl::PointXYZ> cloudtarget;
	cloudsource.push_back(x_axis); cloudsource.push_back(y_axis); cloudsource.push_back(z_axis);
	cloudtarget.push_back(x_axis_t); cloudtarget.push_back(y_axis_t); cloudtarget.push_back(z_axis_t);

	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_est_svd;
	trans_est_svd.estimateRigidTransformation(cloudsource, cloudtarget, T_SVD);
}
void PoseNormalization::Do_transformation()
{
	//cout << this->cloud_input_->height << endl;
	pcl::transformPointCloud(*(this->cloud_input_), this->cloud_output_, T_SVD);
	//save the T_SVD
	
	//save the T_SVD

	//cout << T_SVD << endl;
	//cout << "wuhuuuuuuuuuuuuu" << endl;
	Eigen::Vector4f normal_plane(coefficients_gdplane_(0), coefficients_gdplane_(1), coefficients_gdplane_(2), 0);
	Eigen::Vector4f Point_one_plane(-coefficients_gdplane_(0) * coefficients_gdplane_(3), -coefficients_gdplane_(1) * coefficients_gdplane_(3), -coefficients_gdplane_(2) * coefficients_gdplane_(3), 1);
	normal_plane = T_SVD * normal_plane;
	Point_one_plane = T_SVD * Point_one_plane;
	coefficients_gdplane_t_.head(3) = normal_plane.head(3);
	coefficients_gdplane_t_(3) = -normal_plane.head(3).dot(Point_one_plane.head(3));
	Eigen::Vector4f normal_sysplane(coefficients_symmetry_plane_.values[0], coefficients_symmetry_plane_.values[1],
		coefficients_symmetry_plane_.values[2], 0);
	Eigen::Vector4f Point_one_sysplane(-coefficients_symmetry_plane_.values[0] * coefficients_symmetry_plane_.values[3],
		-coefficients_symmetry_plane_.values[1] * coefficients_symmetry_plane_.values[3],
		-coefficients_symmetry_plane_.values[2] * coefficients_symmetry_plane_.values[3], 1);
	//cout << "wuhuuuuuuuuuuuuu" << endl;
	normal_sysplane = T_SVD * normal_sysplane;
	Point_one_sysplane = T_SVD * Point_one_sysplane;
	coefficients_symmetry_plane_t_.values[0] = normal_sysplane(0);
	coefficients_symmetry_plane_t_.values[1] = normal_sysplane(1);
	coefficients_symmetry_plane_t_.values[2] = normal_sysplane(2);
	coefficients_symmetry_plane_t_.values[3] = -normal_sysplane.head(3).dot(Point_one_plane.head(3));
}

void PoseNormalization::Get_gd_plane(Eigen::VectorXf& coefficients_gdplane)
{
	coefficients_gdplane = coefficients_gdplane_t_;
}
void PoseNormalization::Get_symmetry_plane(pcl::ModelCoefficients& coefficients_symmetry_plane)
{
	coefficients_symmetry_plane = coefficients_symmetry_plane_t_;
}
pcl::PointCloud<pcl::PointXYZRGBA> PoseNormalization::Get_normalizated_data()
{
	return cloud_output_;
}