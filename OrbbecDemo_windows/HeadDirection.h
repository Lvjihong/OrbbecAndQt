#pragma once
#include <iostream>
#include <string>
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/io/png_io.h>

class PigHeadDirectionEstimation {
private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_point_cloud;
	Eigen::Affine3f ViewPose;
	Eigen::Vector3f HeadDirection;
	Eigen::Affine3f world_to_pixel_matrix;////////////////////////////////////
	Eigen::Affine3f Pixel_to_world_matrix;///////////////////////////////////
	pcl::PointXYZRGB head_point;
	pcl::PointXYZRGB body_point;
	int height;
	int width;
	bool input_cloud_ready = false;
	bool ViewPose_ready = false;
	bool rgb_point_cloud_ready = false;
public:
	PigHeadDirectionEstimation();
	PigHeadDirectionEstimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, Eigen::Affine3f ViewPose);
	Eigen::Vector3f getHeadDirection();
	Eigen::Affine3f getViewPose();
	pcl::PointXYZRGB getHeadPoint();
	pcl::PointXYZRGB getbodyPoint();
	void setViewPose(Eigen::Affine3f ViewPose);
	void setinput_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
	//����matrix
	void setmatrix(Eigen::Affine3f world_to_pixel, Eigen::Affine3f Pixel_to_world,int height,int width);
	//��������Ĳ����������ͼ���Ӧ������ͼ��
	void createVirtualImage(int height, int width, int center_x, int center_y, int fx, int fy, std::string filename);
	//��viewer�д���3D��ƽ��
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr create3DPlaneImage();
	//�������峯��
	void caculateHeadDirection(int x_image_head, int y_image_head, int x_image_body, int y_image_body);
	void caculateHeadDirection2(Eigen::Vector3f head_point,Eigen::Vector3i head_color, Eigen::Vector3f body_point, Eigen::Vector3i body_color);
	//�����Ǽ�����߾���
	double DistanceOfPointToLine(pcl::PointXYZRGB* a, pcl::PointXYZRGB* b, pcl::PointXYZRGB* s);
	//���ƺ��߽���
	pcl::PointXYZRGB get_PointCloud_Line_intersect(pcl::PointCloud<pcl::PointXYZRGB>& point_cloud, pcl::PointXYZRGB point1, pcl::PointXYZRGB point2);
};