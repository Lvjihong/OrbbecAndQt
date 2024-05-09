#pragma once

#include "WeightEstimation.h"
#include "ui_MainWindow.h"
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>

#ifdef _DEBUG
#pragma comment(lib, "opencv_world480d.lib")
#else
#pragma comment(lib, "opencv_world480.lib")
#endif

class OrbbecDemo : public QMainWindow {
  Q_OBJECT
 signals:
  void trainWindowShowed(bool flag, QString dir);
  void estimationWindowShowed();

 public:
  OrbbecDemo(QWidget* parent = nullptr);
  OrbbecDemo(const OrbbecDemo& window);
  ~OrbbecDemo();

 private:
  Ui::OrbbecDemoClass ui;
  Train* trainWindow = nullptr;
  WeightEstimation* estimationWindow = nullptr;
  QString dirpath;
};
