#pragma once
#include <QFileDialog>
#include <QWidget>
#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Frame.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "QMessageBox"
#include "Train.h"
#include "libobsensor/hpp/Error.hpp"
#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/StreamProfile.hpp"
#include "ui_WeightEstimation.h"
#include "yolov8_seg.h"
class WeightEstmation : public QWidget {
  Q_OBJECT

 public:
  WeightEstmation(QWidget* parent = nullptr);
  WeightEstmation(const WeightEstmation& window);
  void segment(cv::Mat img);
  void segment(cv::Mat rgb_img, cv::Mat depth_img);
  void estimate_weight(cv::Mat rgb_img, cv::Mat depth_img);
  void estimate_weight(cv::Mat seg_depth_img);
  ~WeightEstmation();

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
  cv::dnn::Net net;

  long long test_time;
};
