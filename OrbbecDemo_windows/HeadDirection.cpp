#include "HeadDirection.h"

PigHeadDirectionEstimation::PigHeadDirectionEstimation() {}

// 初始化

PigHeadDirectionEstimation::PigHeadDirectionEstimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, Eigen::Affine3f ViewPose):input_cloud(input_cloud),ViewPose(ViewPose) {
	input_cloud_ready = true;
	ViewPose_ready = true;

}
// 获取朝向
Eigen::Vector3f PigHeadDirectionEstimation::getHeadDirection() {
	return this->HeadDirection;
}
// 获取位姿
Eigen::Affine3f PigHeadDirectionEstimation::getViewPose() {
	return this->ViewPose;
}
// 获取头部点
pcl::PointXYZRGB PigHeadDirectionEstimation::getHeadPoint() {
	return this->head_point;
}
// 获取身体点
pcl::PointXYZRGB PigHeadDirectionEstimation::getbodyPoint() {
	return this->body_point;
}
// 设置位姿
void PigHeadDirectionEstimation::setViewPose(Eigen::Affine3f ViewPose) {
	this->ViewPose = ViewPose;
	ViewPose_ready = true;
}
// 设置输入电晕
void PigHeadDirectionEstimation::setinput_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud) {
	this->input_cloud = input_cloud;
	input_cloud_ready = true;
}
//设置变形矩阵
void PigHeadDirectionEstimation::setmatrix(Eigen::Affine3f world_to_pixel, Eigen::Affine3f Pixel_to_world,int height,int width)
{
	this->world_to_pixel_matrix = world_to_pixel;
	this->Pixel_to_world_matrix = Pixel_to_world;
	this->height = height;
	this->width = width;
}

//根据输入的参数生成深度图像对应的虚拟图像
void PigHeadDirectionEstimation::createVirtualImage(int height,int width,int center_x,int center_y,int fx,int fy,std::string filename) {
	if (input_cloud_ready&&ViewPose_ready) {
		pcl::RangeImagePlanar::Ptr rangeImage(new pcl::RangeImagePlanar);
		pcl::visualization::RangeImageVisualizer range_image_widget("This is range image");
		rangeImage->createFromPointCloudWithFixedSize(*input_cloud, width, height, center_x, center_y, fx, fy, ViewPose);
		Eigen::Affine3f camera_to_Pixel_matrix(Eigen::Affine3f::Identity());
		camera_to_Pixel_matrix(0, 0) = fx;
		camera_to_Pixel_matrix(1, 1) = fy;
		camera_to_Pixel_matrix(0, 2) = center_x;
		camera_to_Pixel_matrix(1, 2) = center_y;
		world_to_pixel_matrix = camera_to_Pixel_matrix * ViewPose.inverse();
		Pixel_to_world_matrix = world_to_pixel_matrix.inverse();
		pcl::PointXYZRGB temppoint(0, 0, 0, 255, 255, 255);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_point_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
		for (int i = 0; i < height * width; i++) {
			rgb_point_ptr->points.push_back(temppoint);
		}
		float x, y;
		for (int i = 0; i < input_cloud->size(); i++) {
			rangeImage->getImagePoint(input_cloud->points[i].getVector3fMap(), x, y);
			if (x >= 0 && y >= 0 && x < width && y < height) {
				if (rgb_point_ptr->points[(round(x) + round(y) * width)].x==0) {
					rgb_point_ptr->points[(round(x) + round(y) * width)].r = input_cloud->points[i].r;
					rgb_point_ptr->points[(round(x) + round(y) * width)].g = input_cloud->points[i].g;
					rgb_point_ptr->points[(round(x) + round(y) * width)].b = input_cloud->points[i].b;
					rgb_point_ptr->points[(round(x) + round(y) * width)].x = input_cloud->points[i].x;
					rgb_point_ptr->points[(round(x) + round(y) * width)].y = input_cloud->points[i].y;
					rgb_point_ptr->points[(round(x) + round(y) * width)].z = input_cloud->points[i].z;
				}
			}
		}
		const int length_ = 3 * width * height;
		unsigned char * rgb_image_1 =new unsigned char[length_];
		for (int i = 0; i < width * height; i++) {
			rgb_image_1[i * 3] = rgb_point_ptr->points[i].r;
			rgb_image_1[i * 3 + 1] = rgb_point_ptr->points[i].g;
			rgb_image_1[i * 3 + 2] = rgb_point_ptr->points[i].b;
		}
		pcl::io::saveRgbPNGFile(filename, rgb_image_1, rangeImage->width, rangeImage->height);
		this->rgb_point_cloud = rgb_point_ptr;
		this->rgb_point_cloud_ready = true;
		this->height = height;
		this->width = width;
		delete[]rgb_image_1;
		rgb_image_1 = NULL;
	}
		
}
//在viewer中创建3D像平面
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PigHeadDirectionEstimation::create3DPlaneImage() {
	if (rgb_point_cloud_ready) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_3DPlaneImage(new pcl::PointCloud<pcl::PointXYZRGB>);
		Eigen::Vector3f point_temp;
		pcl::PointXYZRGB temp_point(0, 0, 0, 0, 0, 0);
		for (int i = 0; i < this->height * this->width; i++) {
			if (!(rgb_point_cloud->points[i].x == 0 && rgb_point_cloud->points[i].y == 0
				&& rgb_point_cloud->points[i].z == 0)) {
				point_temp[0] = i % this->width;
				point_temp[1] = i / this->width;
				point_temp[2] = 1;
				point_temp = Pixel_to_world_matrix * point_temp;
				temp_point.x = point_temp[0];
				temp_point.y = point_temp[1];
				temp_point.z = point_temp[2];
				temp_point.r = rgb_point_cloud->points[i].r;
				temp_point.g = rgb_point_cloud->points[i].g;
				temp_point.b = rgb_point_cloud->points[i].b;
				rgb_3DPlaneImage->push_back(temp_point);
			}
		}
		return rgb_3DPlaneImage;
	}
	return NULL;
}
// 计算朝向,最终返回头部点和身体点
void PigHeadDirectionEstimation::caculateHeadDirection(int x_image_head, int y_image_head, int x_image_body, int y_image_body) {
	Eigen::Vector3f point_caculate_head(x_image_head, y_image_head, 1);//没有深度的点
	Eigen::Vector3f point_caculate_body(x_image_body, y_image_body, 1);//没有深度的点
	point_caculate_head = Pixel_to_world_matrix * point_caculate_head;
	point_caculate_body = Pixel_to_world_matrix * point_caculate_body;
	pcl::PointXYZRGB LinePoint_a_head;
	pcl::PointXYZRGB LinePoint_b_cam;
	pcl::PointXYZRGB LinePoint_a_body;
	LinePoint_a_body.x = point_caculate_body[0];
	LinePoint_a_body.y = point_caculate_body[1];
	LinePoint_a_body.z = point_caculate_body[2];
	LinePoint_a_head.x = point_caculate_head[0];
	LinePoint_a_head.y = point_caculate_head[1];
	LinePoint_a_head.z = point_caculate_head[2];
	LinePoint_b_cam.x = this->ViewPose(0, 3);
	LinePoint_b_cam.y = this->ViewPose(1, 3);
	LinePoint_b_cam.z = this->ViewPose(2, 3);
	this->head_point = get_PointCloud_Line_intersect(*rgb_point_cloud, LinePoint_b_cam, LinePoint_a_head);//头部点
	this->body_point = get_PointCloud_Line_intersect(*rgb_point_cloud, LinePoint_b_cam, LinePoint_a_body);//身体点

}
//有深度图时计算朝向，返回头部点和身体点
void PigHeadDirectionEstimation::caculateHeadDirection2(Eigen::Vector3f head_point, Eigen::Vector3i head_color, Eigen::Vector3f body_point, Eigen::Vector3i body_color)
{
	float head_dep = head_point[2] /1000;
	float body_dep= body_point[2] / 1000;
	Eigen::Vector3f point_caculate_head(head_point[0]*head_dep, head_point[1]*head_dep, head_dep);
	Eigen::Vector3f point_caculate_body(body_point[0]*body_dep, body_point[1]*body_dep, body_dep);
	point_caculate_head = Pixel_to_world_matrix * point_caculate_head;
	point_caculate_body = Pixel_to_world_matrix * point_caculate_body;
	this->head_point.x = point_caculate_head[0];
	this->head_point.y = point_caculate_head[1];
	this->head_point.z = point_caculate_head[2];
	this->head_point.r = head_color[0];
	this->head_point.g = head_color[1];
	this->head_point.b = head_color[2];

	this->body_point.x = point_caculate_body[0];
	this->body_point.y = point_caculate_body[1];
	this->body_point.z = point_caculate_body[2];
	this->body_point.r = body_color[0];
	this->body_point.g = body_color[1];
	this->body_point.b = body_color[2];


}

//以下是计算点线距离
double PigHeadDirectionEstimation::DistanceOfPointToLine(pcl::PointXYZRGB* a, pcl::PointXYZRGB* b, pcl::PointXYZRGB* s)
{
	double ab = sqrt(pow((a->x - b->x), 2.0) + pow((a->y - b->y), 2.0) + pow((a->z - b->z), 2.0));
	double as = sqrt(pow((a->x - s->x), 2.0) + pow((a->y - s->y), 2.0) + pow((a->z - s->z), 2.0));
	double bs = sqrt(pow((s->x - b->x), 2.0) + pow((s->y - b->y), 2.0) + pow((s->z - b->z), 2.0));
	double cos_A = (pow(as, 2.0) + pow(ab, 2.0) - pow(bs, 2.0)) / (2 * ab * as);
	double sin_A = sqrt(1 - pow(cos_A, 2.0));
	return as * sin_A;
}
//点云和线焦点
pcl::PointXYZRGB PigHeadDirectionEstimation::get_PointCloud_Line_intersect(pcl::PointCloud<pcl::PointXYZRGB>& point_cloud, pcl::PointXYZRGB point1, pcl::PointXYZRGB point2) {
	pcl::PointXYZRGB result_point;
	double dist = DBL_MAX;
	double temp = 0;
	for (int i = 0; i < point_cloud.size(); i++) {
		temp = DistanceOfPointToLine(&point1, &point2, &point_cloud.points[i]);
		if (dist > temp) {
			dist = temp;
			result_point.x = point_cloud.points[i].x;
			result_point.y = point_cloud.points[i].y;
			result_point.z = point_cloud.points[i].z;

		}
	}
	return result_point;
}

