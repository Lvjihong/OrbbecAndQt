#pragma once
#define NOMINMAX
#undef slots
#include <torch/script.h>
#include <torch/torch.h>
#define slots Q_SLOTS

#include <time.h>

#include "Train.h"
#include "ui_WeightEstimation.h"
#include <QMessageBox>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#include "yolov8_seg.h"
#include "yolov8.h"
#include "HeadDirection.h"
#include "Segemention.h"
#include "SymmetricalPlane.h"
#include "PoseNormalization.h"

class WeightEstimation : public QWidget {
  Q_OBJECT

 public:
  WeightEstimation(QWidget* parent = nullptr);
  WeightEstimation(const WeightEstimation& window);
  void segment(cv::Mat rgb_img, cv::Mat depth_img);
  void estimate_weight_by_img_plus(cv::Mat rgb_img, cv::Mat depth_img);
  void init_models();
  void PreProcess(const cv::Mat& image, cv::Mat& image_blob);
  ~WeightEstimation();

 signals:
  void show_img(cv::Mat);

 public slots:
  void showDepth();
  void showImg(cv::Mat img);

 private:
  Ui::WeightEstimation ui;
  ob::Pipeline pipe;
  std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
  Yolov8Seg task_segment;
  Yolov8 task_detect;
  cv::dnn::Net seg_net;
  cv::dnn::Net estimate_net;
  cv::dnn::Net detect_net;
  torch::jit::script::Module module_weight_estimation;
  torch::jit::script::Module module_find_head;
  long long test_time;
  torch::Device device_cpu = torch::Device(torch::kCPU);  //定义cpu设备
};
