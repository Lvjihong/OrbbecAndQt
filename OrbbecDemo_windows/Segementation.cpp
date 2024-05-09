#include "Segemention.h"
SegementationPig::SegementationPig() {
}
SegementationPig::SegementationPig(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, Eigen::Affine3f ViewPose) {
    this->input_cloud = input_cloud;
    this->ViewPose = ViewPose;
    input_cloud_ready = true;
	ViewPose_ready = true;
}
void SegementationPig::setViewPose(Eigen::Affine3f ViewPose) {
	this->ViewPose = ViewPose;
	ViewPose_ready = true;
}
void SegementationPig::setinput_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud) {
	this->input_cloud = input_cloud;
	input_cloud_ready = true;
}

//利用mask来分割猪体点云


// 将输入的点云去除其中的平面，并根据目标检测的结果判断目标物体的点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr SegementationPig::Handle_input_cloud_with_DL(pcl::PointXYZRGB point_real) {
    //// 下采样，1cm，应该不需要
    //pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    //vg.setInputCloud(this->input_cloud);
    //vg.setLeafSize(0.01f, 0.01f, 0.01f);
    //vg.filter(*cloud_filtered);
    //std::cout << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl; //下采样

    //非下采样的输出////////////
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    /*for (int i = 0; i < this->input_cloud->points.size(); i++)
    {
        pcl::PointXYZRGB temp;
        temp.x = this->input_cloud->points[i].x;
        temp.y = this->input_cloud->points[i].y;
        temp.z = this->input_cloud->points[i].z;
        temp.r = this->input_cloud->points[i].r;
        temp.g = this->input_cloud->points[i].g;
        temp.b = this->input_cloud->points[i].b;
        cloud_filtered->points.push_back(temp);
    }*/
    pcl::copyPointCloud( *(this->input_cloud), *cloud_filtered);
    cloud_filtered->height = 1;
    cloud_filtered->width = cloud_filtered->points.size();
    //非下采样的输出////////////

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>());
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);//最大迭代次数
    seg.setDistanceThreshold(0.05);

    int i = 0, nr_points = (int)cloud_filtered->size();//体素滤波后的点云
    //int idc = 1;////////////////////////////////////////debug
    while (cloud_filtered->size() > 0.3 * nr_points)//分割最重要的部分
    {
        // 分割最大的平面
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() < 15000)//平面值小于15000
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        
        // 提取点云中的平面
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);//平面点索引

        //================debug
       
        /*std::string path = "./testfile/plane_" + std::to_string(idc) + ".pcd";
        extract.setNegative(false);
        extract.filter(*cloud_plane);
        pcl::io::savePCDFile(path, *cloud_plane);
        idc++;*/
        //================debug
       
        // 去除平面，保留剩余部分
        extract.setNegative(true);
        extract.filter(*(this->input_cloud));
        *cloud_filtered = *(this->input_cloud);
        
    }
    //==================debug
    /*std::string temp_path = "C:\\Users\\15207\\Desktop\\bodysize\\pig_weight\\pythoncode\\pose-normlization\\myproject\\pose-normlization\\testfile\\filter_1.pcd";
    pcl::io::savePCDFile(temp_path, *cloud_filtered);*/


    //=================debug

    // 根据距离身体点的远近判断哪一个点云聚类为目标物体
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_filtered);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.035);
    ec.setMinClusterSize(3000);
    ec.setMaxClusterSize(250000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);
    double dis_point = DBL_MAX;
    int flag= 0;
    double dis_temp = 0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_return;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        flag = 0;
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            cloud_cluster->push_back((*cloud_filtered)[*pit]); 
            dis_temp = sqrt(pow(((*cloud_filtered)[*pit].x - point_real.x), 2.0) + pow(((*cloud_filtered)[*pit].y - point_real.y), 2.0) + pow(((*cloud_filtered)[*pit].z - point_real.z), 2.0));
            if (dis_point>dis_temp) {
                flag = 1;
                dis_point = dis_temp;
            }
        }
        if (flag==1) {
            cloud_return = cloud_cluster;
        }
    }
    cloud_return->width = cloud_return->size();
    cloud_return->height = 1;
    cloud_return->is_dense = true;
    std::cout << "PointCloud representing the Cluster: " << cloud_return->size() << " data points." << std::endl;
    return cloud_return;
}
// 计算地面法向量
void SegementationPig::create_ground_plane_vector(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputcloud,pcl::PointXYZRGB point_real) {
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_ground(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(inputcloud));
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model_ground);
    gd_plane_inliers_src_.reset(new pcl::PointIndices);
    ransac.setMaxIterations(10000);
    ransac.setDistanceThreshold(0.026);
    ransac.computeModel();
    ransac.getInliers(gd_plane_inliers_src_->indices);
    ransac.getModelCoefficients(up_ground_vec_1);

//save the ground debug//////////////////////////////////
    
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr final(new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::copyPointCloud(*inputcloud, gd_plane_inliers_src_->indices, *final);
    //std::string test_file = "./testfile/grd.pcd";
    //pcl::io::savePCDFile(test_file, *final);

//save the ground///////////////////////////////////////

    Eigen::Vector3f pig_point_plane_point;
    pig_point_plane_point[0] = point_real.x;
    pig_point_plane_point[1] = point_real.y;
    pig_point_plane_point[2] = point_real.z+ (up_ground_vec_1[3]/ up_ground_vec_1[2]);
    if (pig_point_plane_point[0]* up_ground_vec_1[0]+ pig_point_plane_point[1] * up_ground_vec_1[1]+ pig_point_plane_point[2] * up_ground_vec_1[2] < 0) {
        up_ground_vec_1 = -up_ground_vec_1;
    }
    
}
