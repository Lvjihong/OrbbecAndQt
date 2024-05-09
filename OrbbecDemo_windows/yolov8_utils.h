#pragma once
#include <io.h>
#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <fstream>

struct OutputSeg {
  int id;            //结果类别id
  float confidence;  //结果的置信度
  cv::Rect box;      //包围框xyhw类型
  cv::Mat boxMask;   // mask
};

struct MaskParams {
  int netWidth = 640;
  int netHeight = 640;
  float maskThreshold = 0.5;
  cv::Size srcImgShape;
  cv::Vec4d params;
};

bool CheckModelPath(std::string modelPath);
bool CheckParams(int netHeight, int netWidth, const int* netStride,
                 int strideSize);

cv::Mat DrawPred(cv::Mat& img, std::vector<OutputSeg> result,
                 std::vector<std::string> classNames,
                 std::vector<cv::Scalar> color,
                 bool isVideo = false);  //画出预测结果
std::vector<cv::Mat> DrawPred(cv::Mat& img, cv::Mat depth_img,
                              std::vector<OutputSeg> result,
                              std::vector<std::string> classNames,
                              std::vector<cv::Scalar> color,
                              bool isVideo = false);  //画出预测结果
void DrawPred_box(cv::Mat& img, std::vector<OutputSeg> result,
                  std::vector<std::string> classNames, std::vector<cv::Scalar> color,
                  bool isVideo = false);
void LetterBox(const cv::Mat& image, cv::Mat& outImage,
               cv::Vec4d& params,  //[ratio_x,ratio_y,dw,dh]
               const cv::Size& newShape = cv::Size(640, 640),
               bool autoShape = false, bool scaleFill = false,
               bool scaleUp = true, int stride = 32,
               const cv::Scalar& color = cv::Scalar(114, 114, 114));

void GetMask(const cv::Mat& maskProposals, const cv::Mat& maskProtos,
             std::vector<OutputSeg>& output, const MaskParams& maskParams);
void GetMask2(const cv::Mat& maskProposals, const cv::Mat& maskProtos,
              OutputSeg& output, const MaskParams& maskParams);
