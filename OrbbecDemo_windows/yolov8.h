#pragma once
#include "yolov8_utils.h"

class Yolov8 {
public:
	Yolov8() {
	}
	~Yolov8() {}

	bool ReadModel(cv::dnn::Net& net, std::string& netPath, bool isCuda);
	bool Detect(cv::Mat& srcImg, cv::dnn::Net& net, std::vector<OutputSeg>& output);

	int _netWidth = 640;   //ONNXͼƬ������
	int _netHeight = 640;  //ONNXͼƬ����߶�


	//swswssssws
	std::vector<std::string> _className = {"body","head","hip"};
private:
	float _classThreshold = 0.25;
	float _nmsThreshold = 0.45;
};
