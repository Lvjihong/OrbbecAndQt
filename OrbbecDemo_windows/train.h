#pragma once
#include <qdirmodel.h>
#include <qpushbutton.h>

#include <QCloseEvent>
#include <QFileDialog>
#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Frame.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "InputWeightDialog.h"
#include "libobsensor/hpp/Error.hpp"
#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/StreamProfile.hpp"
#include "ui_Train.h"

class Train : public QWidget {
  Q_OBJECT

 public:
  Train(const QString dirPath, QWidget* parent = nullptr);
  Train(const Train& trainWindow);
  ~Train();
  void closeEvent(QCloseEvent* e);
  static std::vector<cv::Mat> frame2Mat(
      const std::shared_ptr<ob::VideoFrame>& frame);
  static QImage mat2QImage(cv::Mat cvImg);

 public slots:
  void updateTreeView();
  void saveOrShowAll(bool flag, QString rootDirPath);

 private:
  Ui::TrainWindow ui;
  ob::Pipeline pipe;
  std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
  int depthCount;
  int colorCount;
  QString rootDirPath;
  bool isSave = false;
  QDirModel* model;
};
