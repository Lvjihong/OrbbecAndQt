#pragma once
#include "yolov8_utils.h"

class Yolov8Seg {
public:
	Yolov8Seg() {
	}
	~Yolov8Seg() {}

	bool ReadModel(cv::dnn::Net& net, std::string& netPath, bool isCuda);
	bool Detect(cv::Mat& srcImg, cv::dnn::Net& net, std::vector<OutputSeg>& output);

	//��������Լ���ģ����Ҫ�޸Ĵ���
	std::vector<std::string> _className = {"pig"};
	int _netWidth = 640;   //ONNXͼƬ������
	int _netHeight = 640;  //ONNXͼƬ����߶�

private:
	float _classThreshold = 0.25;//���Ŷ���ֵ
	float _nmsThreshold = 0.45;
	float _maskThreshold = 0.5;
};
