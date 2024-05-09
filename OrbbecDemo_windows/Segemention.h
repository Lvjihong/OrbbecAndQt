#pragma once
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/io/png_io.h>
#include <string>
class SegementationPig {
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud;
    bool input_cloud_ready = false;
    Eigen::Affine3f ViewPose;
    bool ViewPose_ready = false;
public:

    Eigen::VectorXf up_ground_vec_1;//是否所有提出的都是地面
    pcl::PointIndices::Ptr gd_plane_inliers_src_;//这个索引基本没什么用
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_plane;
public:
    SegementationPig();
    SegementationPig(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, Eigen::Affine3f ViewPose);
    void setViewPose(Eigen::Affine3f ViewPose);
    void setinput_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
    //输出分割后的猪体点云
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg_pig_cloud(std::string dep_file, std::string rgb_file, std::string mask_file);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Handle_input_cloud_with_DL(pcl::PointXYZRGB point_real);
    //生成地面向上矢量
    void create_ground_plane_vector(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud,pcl::PointXYZRGB point_real);
};