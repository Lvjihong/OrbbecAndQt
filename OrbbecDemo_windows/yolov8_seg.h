#pragma once
#include "yolov8_utils.h"

class Yolov8Seg {
public:
	Yolov8Seg() {
	}
	~Yolov8Seg() {}

	bool ReadModel(cv::dnn::Net& net, std::string& netPath, bool isCuda);
	bool Detect(cv::Mat& srcImg, cv::dnn::Net& net, std::vector<OutputSeg>& output);

	//类别名，自己的模型需要修改此项
	std::vector<std::string> _className = {"pig"};
	int _netWidth = 640;   //ONNX图片输入宽度
	int _netHeight = 640;  //ONNX图片输入高度

private:
	float _classThreshold = 0.25;//置信度阈值
	float _nmsThreshold = 0.45;
	float _maskThreshold = 0.5;
};
