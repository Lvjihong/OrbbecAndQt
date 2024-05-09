#include "SymmetricalPlane.h"

SymmetricalPlane::SymmetricalPlane():search_radius_(0.1),biggest_profile_input_(false),up_vector_input_(false)
{
	biggest_profile_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
	current_samples_for_eachiteration_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
}
void SymmetricalPlane::Reset()
{
	biggest_profile_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
	current_samples_for_eachiteration_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
	biggest_profile_input_ = false;
	up_vector_input_ = false;
}
void SymmetricalPlane::SetBiggestProfile(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr profile)
{
	biggest_profile_1 = getColoredCloud(profile, 155, 155, 155);
	int result_cloud_size = 0;
	Eigen::Vector3f major_vector, middle_vector, minor_vector, mass_center;
	pcl::PointXYZRGBA min_point_AABB;//AABB与坐标轴对齐的包围盒
	pcl::PointXYZRGBA max_point_AABB;
	pcl::PointXYZRGBA min_point_OBB;//有向包围盒
	pcl::PointXYZRGBA max_point_OBB;
	pcl::PointXYZRGBA position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	float interval_between_slice_forporfile_ = 0.02;////////////////////////////////////////0.02
	float threshold_plane_thinkness_forprofile_ = 0.02;/////////////////////////////////////0.04
	pcl::MomentOfInertiaEstimation <pcl::PointXYZRGBA> feature_extractor;
	feature_extractor.setInputCloud(profile);
	feature_extractor.compute();
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);//计算特征向量
	feature_extractor.getMassCenter(mass_center);//计算质心
	feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	float max_height = abs(max_point_OBB.z - min_point_OBB.z) * 4;//这里有点问题
	//debug
	//cout << "盒子大小为：" << max_point_OBB.z - min_point_OBB.z << "\n" << max_point_OBB.y - min_point_OBB.y << "\n" << max_point_OBB.x - min_point_OBB.x;
	//debug
	for (int i = 0; i < ceil(max_height / interval_between_slice_forporfile_); i++)
	{
		int sample_cloud_size;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp_sample_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		Eigen::Vector4f temp_virtual_plane = Constructing_vplane_along_vector_moved(up_normal_plane_, up_normal_plane_.head(4), i * interval_between_slice_forporfile_);
		temp_sample_cloud = Intersection_plane_with_object_computing(temp_virtual_plane, threshold_plane_thinkness_forprofile_, profile);
		sample_cloud_size = temp_sample_cloud->size();
		if (sample_cloud_size > result_cloud_size)//找最大的
		{

			result_cloud_size = sample_cloud_size;
			biggest_profile_ = temp_sample_cloud;
		}
	}

	//debug////////////////////////////////////////

	/*std::string save_file = "./testfile/bigest.pcd";
	pcl::io::savePCDFile(save_file, *biggest_profile_);*/

	//debug///////////////////////////////////////


	//*biggest_profile_ = *profile;
}
void SymmetricalPlane::SetUpVector(Eigen::VectorXf up_normal_plane)
{
	up_normal_plane_ = up_normal_plane;

}
void SymmetricalPlane::SetSearchRadius(float radius)
{
	search_radius_ = radius;
}
pcl::ModelCoefficients SymmetricalPlane::get_estimated_plane()
{
	return symmetry_plane_estimated_coeff;
}
pcl::PointXYZRGBA SymmetricalPlane::get_point_onplane()
{
	return result_point_onplane_;
}
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr SymmetricalPlane::getColoredCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_, float r, float g, float b)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colored_cloud;
	colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGBA>)->makeShared();
	colored_cloud->width = input_->width;
	colored_cloud->height = input_->height;
	colored_cloud->is_dense = input_->is_dense;
	for (size_t i_point = 0; i_point < input_->points.size(); i_point++)
	{
		pcl::PointXYZRGBA point;
		point.x = *(input_->points[i_point].data);
		point.y = *(input_->points[i_point].data + 1);
		point.z = *(input_->points[i_point].data + 2);
		point.r = r;
		point.g = g;
		point.b = b;
		colored_cloud->points.push_back(point);
	}
	return (colored_cloud);
}
void SymmetricalPlane::Do_Computing_thread()
{
	Computing_profile_longest_direction();
	float max_rate = 0;
	float process_vote_rate = 0;
	int convergence_count = 0;
	int Max_convergence_count = 3;//迭代中比例不变时的最大重复次数3
	pcl::PointXYZRGBA max_rate_point;
	pcl::Normal max_rate_normal;  //rate  result plane coff
	int cloud_profiles_size = biggest_profile_->points.size();
	//ransac
	/*pcl::visualization::PCLVisualizer viewer ("Visualization of procedure of voting");
	this->biggest_profile_1->width = this->biggest_profile_1->size();
	this->biggest_profile_1->height = 1;
	this->biggest_profile_1->is_dense = false;
	viewer.addCoordinateSystem(0.1);
	viewer.setBackgroundColor(1,1,1);
	viewer.addPointCloud(getColoredCloud(this->biggest_profile_1,155,155,155),"biggest_profile_");*/

	for (int k = 0; k < cloud_profiles_size; k = k + cloud_profiles_size / 10)
	{


		float vote_rate;
		float filter_dot_result;
		int vote_count;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sample_cloud_foriteration(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointXYZRGBA sample_random_point_1, sample_random_point_2;
		Eigen::Vector3f v_sample_random_point_1;
		//迭代多次后比例不提升，退出
		if (process_vote_rate == max_rate)
		{
			convergence_count++;
			if (convergence_count >= Max_convergence_count)
				break;
		}
		else
			convergence_count = 0;
		process_vote_rate = max_rate;
		sample_random_point_1 = biggest_profile_->points[k];
		v_sample_random_point_1 = sample_random_point_1.getArray3fMap();
		sample_cloud_foriteration = Search_current_samples_foriteration(search_radius_, sample_random_point_1);
		std::stringstream ss_sample_cloud_foriteration;
		ss_sample_cloud_foriteration << k;
		ss_sample_cloud_foriteration << "scfi";

		//viewer.addPointCloud(getColoredCloud(sample_cloud_foriteration,static_cast<unsigned char> (rand () % 256),static_cast<unsigned char> (rand () % 256),static_cast<unsigned char> (rand () % 256)),ss_sample_cloud_foriteration.str());
		//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,ss_sample_cloud_foriteration.str());
		//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.8,ss_sample_cloud_foriteration.str());
		for (int i = 0; i < sample_cloud_foriteration->points.size(); i++)
		{
			pcl::PointXYZRGBA test_plane_point;   // middle of the two random pointss
			pcl::Normal verification_plane_normal;
			Eigen::Vector3f v_sample_random_point_2;
			Eigen::Vector3f v_verification_plane_normal;
			v_sample_random_point_2 = sample_cloud_foriteration->points[i].getArray3fMap();
			//假设所需的第二个点
			v_verification_plane_normal = v_sample_random_point_1 - v_sample_random_point_2;
			v_verification_plane_normal.normalize();
			//构造假设模型的法线值
			verification_plane_normal.normal_x = v_verification_plane_normal(0);
			verification_plane_normal.normal_y = v_verification_plane_normal(1);
			verification_plane_normal.normal_z = v_verification_plane_normal(2);
			test_plane_point.x = (v_sample_random_point_1(0) + v_sample_random_point_2(0)) / 2;
			test_plane_point.y = (v_sample_random_point_1(1) + v_sample_random_point_2(1)) / 2;
			test_plane_point.z = (v_sample_random_point_1(2) + v_sample_random_point_2(2)) / 2;
			//构造假设模型的点值
			//得到当前模型的票数
			if (i % 200 == 0)
			{
				std::stringstream virtual_plane;
				virtual_plane << i;
				virtual_plane << k;
				virtual_plane << "virtual_plane";
				pcl::ModelCoefficients tmp_plane_coeff;
				Eigen::Vector4f tmp_virtual_plane;
				tmp_virtual_plane.head(3) = v_verification_plane_normal;
				tmp_virtual_plane(3) = -(v_verification_plane_normal(0) * test_plane_point.x + v_verification_plane_normal(1) * test_plane_point.y + v_verification_plane_normal(2) * test_plane_point.z);
				tmp_plane_coeff.values.insert(tmp_plane_coeff.values.begin(), tmp_virtual_plane.data(), tmp_virtual_plane.data() + 4);
				//viewer.addPlane(tmp_plane_coeff,test_plane_point.x,test_plane_point.y,test_plane_point.z,virtual_plane.str());
				//viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.5,0.5,0.5,virtual_plane.str());
				//viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.4,virtual_plane.str());
				virtual_plane << k;
				virtual_plane << i;
				//viewer.addLine<pcl::PointXYZRGBA,pcl::PointXYZRGBA>(sample_random_point_1, sample_cloud_foriteration->points[i],1.0,0,0,virtual_plane.str());
				//viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,4,virtual_plane.str());
				virtual_plane << "sh";

				virtual_plane << "sh";
			}
			vote_count = Ransac_Test(search_radius_, biggest_profile_, test_plane_point, verification_plane_normal);
			vote_rate = (float)vote_count / (float)biggest_profile_->size();
			if (vote_rate > max_rate)
			{
				max_rate = vote_rate;
				max_rate_point = test_plane_point;
				max_rate_normal = verification_plane_normal;
			}
			//循环下一个假设
		}
		std::cout << k << endl;
		std::cout << max_rate << endl;
	}


	ratio_sample_consensus_ = max_rate;
	result_point_onplane_ = max_rate_point;
	result_normal_forplane_ = max_rate_normal;
	symmetry_plane_estimated_coeff.values.resize(4);
	symmetry_plane_estimated_coeff.values[0] = result_normal_forplane_.normal_x;
	symmetry_plane_estimated_coeff.values[1] = result_normal_forplane_.normal_y;
	symmetry_plane_estimated_coeff.values[2] = result_normal_forplane_.normal_z;
	symmetry_plane_estimated_coeff.values[3] = -(result_normal_forplane_.normal_x * result_point_onplane_.x + result_normal_forplane_.normal_y * result_point_onplane_.y + result_normal_forplane_.normal_z * result_point_onplane_.z);
	//viewer.addPlane(symmetry_plane_estimated_coeff,result_point_onplane_.x,result_point_onplane_.y,result_point_onplane_.z,"sys_plane");
	//viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.9,0.9,0.2,"sys_plane");
	//viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.6,"sys_plane");
	//viewer.spin();
	//return viewer;
}
void SymmetricalPlane::Computing_profile_longest_direction()
{
	pcl::MomentOfInertiaEstimation <pcl::PointXYZRGBA> feature_extractor;
	feature_extractor.setInputCloud(biggest_profile_);
	feature_extractor.compute();
	feature_extractor.getEigenVectors(PCA_X_Vector, PCA_Y_Vector, PCA_Z_Vector);
	feature_extractor.getMassCenter(biggest_profile_mass_center);
	Eigen::Vector3f normal;
	Eigen::Vector3f tmp = up_normal_plane_.head(3);
	normal = tmp.cross(PCA_X_Vector);
	normal.normalize();
	normal_for_contructing_plane_.normal_x = normal(0);
	normal_for_contructing_plane_.normal_y = normal(1);
	normal_for_contructing_plane_.normal_z = normal(2);
}
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr SymmetricalPlane::Search_current_samples_foriteration(float search_radius, pcl::PointXYZRGBA current_point)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr RadiusSearch_result(new pcl::PointCloud<pcl::PointXYZRGBA>);
	std::vector<int> inliers; //inliers
	pcl::KdTreeFLANN<pcl::PointXYZRGBA> sample_cloud_kdtree;
	std::vector<int> sample_cloud_pointIdxRadiusSearch;
	std::vector<float> sample_cloud_pointRadiusSquaredDistance;
	pcl::PointXYZRGBA MR_point, plane_point;
	plane_point = current_point;
	plane_point.getVector3fMap() = biggest_profile_mass_center;
	MR_point = MirrorReflection_onePoint_along_plane(current_point, plane_point, normal_for_contructing_plane_);
	sample_cloud_kdtree.setInputCloud(biggest_profile_);
	sample_cloud_pointIdxRadiusSearch.clear();
	sample_cloud_pointRadiusSquaredDistance.clear();
	sample_cloud_kdtree.radiusSearch(MR_point, search_radius, sample_cloud_pointIdxRadiusSearch, sample_cloud_pointRadiusSquaredDistance);
	if (sample_cloud_pointIdxRadiusSearch.size() > 1)
	{
		for (int i = 0; i < sample_cloud_pointIdxRadiusSearch.size(); i++)
		{
			int point_num;
			pcl::PointXYZRGBA current_point;
			point_num = sample_cloud_pointIdxRadiusSearch[i];
			current_point = biggest_profile_->points[point_num];
			RadiusSearch_result->points.push_back(current_point);
		}
	}
	return RadiusSearch_result;
}
pcl::PointXYZRGBA SymmetricalPlane::MirrorReflection_onePoint_along_plane(pcl::PointXYZRGBA sample_point, pcl::PointXYZRGBA plane_point, pcl::Normal plane_normal)
{
	float distance_to_plane;
	pcl::PointXYZRGBA MR_point = sample_point;
	Eigen::Vector3f v_sample_point, v_MR_point, plane_pedal;
	Eigen::Vector4f v_plane_coefficients;
	Eigen::Vector3f vertical;  //  vertical normal
	v_sample_point = sample_point.getArray3fMap();
	PointToPlaneDistance(sample_point, plane_point, plane_normal, distance_to_plane, plane_pedal);
	vertical = plane_pedal - v_sample_point;
	vertical.normalize();
	v_MR_point = v_sample_point + (2 * distance_to_plane) * vertical;  // compute  MR_point
	MR_point.x = v_MR_point.x();
	MR_point.y = v_MR_point.y();
	MR_point.z = v_MR_point.z();
	return MR_point;
}
void SymmetricalPlane::PointToPlaneDistance(pcl::PointXYZRGBA sample_point, pcl::PointXYZRGBA plane_point, pcl::Normal plane_normal, float& distance_return, Eigen::Vector3f& pedal_return)
{
	float sb, sn, sd;
	Eigen::Vector3f normal_plane;
	Eigen::Vector3f v_sample_point, v_plane_point;
	v_sample_point = sample_point.getArray3fMap();
	v_plane_point = plane_point.getArray3fMap();
	normal_plane = plane_normal.getNormalVector3fMap();
	sn = -normal_plane.dot(v_sample_point - v_plane_point);
	sd = sqrt(normal_plane.dot(normal_plane));
	sb = sn / sd;
	pedal_return = v_sample_point + sb * normal_plane;
	distance_return = sqrt((v_sample_point - pedal_return).dot(v_sample_point - pedal_return));
}
int SymmetricalPlane::Ransac_Test(float search_radius, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_profiles, pcl::PointXYZRGBA plane_point, pcl::Normal plane_normal)
{
	float verifacation_threshold = 0.25 * search_radius;
	int count = 0;  // save the number of inliers
	std::vector<int> inliers; //inliers
	pcl::KdTreeFLANN<pcl::PointXYZRGBA> cloud_profiles_kdtree;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	cloud_profiles_kdtree.setInputCloud(cloud_profiles);  // kd done
	//构造样本集的kd树
	for (size_t i = 0; i < cloud_profiles->points.size(); ++i)   // search each point
	{
		float current_point_distance;
		pcl::PointXYZRGBA MR_point;
		pcl::PointXYZRGBA sample_point;
		sample_point = cloud_profiles->points[i];
		MR_point = MirrorReflection_onePoint_along_plane(sample_point, plane_point, plane_normal);
		//构造对称点对
		pointIdxRadiusSearch.clear();
		pointRadiusSquaredDistance.clear();
		cloud_profiles_kdtree.radiusSearch(MR_point, verifacation_threshold, pointIdxRadiusSearch, pointRadiusSquaredDistance);
		//验证当前对称点对称是否与模型一致
		if (pointIdxRadiusSearch.size() > 1)
		{
			inliers.push_back(i);
			count++;
		}
	}
	return count;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr SymmetricalPlane::Intersection_plane_with_object_computing(Eigen::Vector4f
	plane, float threshold, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr profile)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr search_result_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA>::Ptr
		dit(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA>(profile));
	Eigen::Vector4f coefficients = plane;
	std::vector<int> inliers;
	dit->selectWithinDistance(coefficients, threshold, inliers);
	pcl::copyPointCloud<pcl::PointXYZRGBA>(*profile, inliers, *search_result_cloud);
	return search_result_cloud;
}


Eigen::Vector4f SymmetricalPlane::Constructing_vplane_along_vector_moved(Eigen::Vector4f
	vector, Eigen::Vector4f plane_old, float distance_moved)
{
	Eigen::Vector4f new_plane;
	//constructing transformation matrix
	Eigen::Matrix4f Tr = Eigen::Matrix4f::Identity();
	//pedal point lying on old plane
	Eigen::Vector4f Point_one_plane(-plane_old(0) * plane_old(3), -plane_old(1) * plane_old(3), -plane_old(2) * plane_old(3), 1);//平面上的点坐标
	//move pdeal along vector
	//vector.normalize(); 
	Eigen::Vector4f Point_one_plane_new = Point_one_plane + distance_moved * vector;
	Eigen::Vector4f translation = distance_moved * vector;
	Tr.block<3, 1>(0, 3) = translation.head(3);
	Eigen::Vector4f normal_plane(plane_old(0), plane_old(1), plane_old(2), 0);
	normal_plane = (Tr.inverse()).transpose() * normal_plane;
	normal_plane.normalize();
	Point_one_plane = Tr * Point_one_plane;
	new_plane.head(3) = normal_plane.head(3);
	new_plane(3) = -normal_plane.head(3).dot(Point_one_plane.head(3));
	return new_plane;
}
