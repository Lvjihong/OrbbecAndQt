#pragma once
#include <QFileDialog>
#include <QWidget>
#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Frame.hpp>
#include <opencv2/opencv.hpp>

#include "Train.h"
#include "libobsensor/hpp/Error.hpp"
#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/StreamProfile.hpp"
#include "ui_WeightEstimation.h"
class WeightEstmation : public QWidget {
  Q_OBJECT

 public:
  WeightEstmation(QWidget* parent = nullptr);
  WeightEstmation(const WeightEstmation& window);
  ~WeightEstmation();

 public slots:
  void showDepth();

 private:
  Ui::WeightEstimation ui;
  ob::Pipeline pipe;
  std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
};
