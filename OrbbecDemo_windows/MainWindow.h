#pragma once

#include <qpushbutton.h>
#include <QFileDialog>
#include <QtWidgets/QMainWindow>
#include "Train.h"
#include "WeightEstmation.h"
#include "ui_MainWindow.h"

#ifdef _DEBUG
#pragma comment(lib, "opencv_world480d.lib")
#else
#pragma comment(lib, "opencv_world480.lib")
#endif
class OrbbecDemo : public QMainWindow {
  Q_OBJECT
 signals:
  void windowShowed(bool flag, QString dir);

 public:
  OrbbecDemo(QWidget* parent = nullptr);
  OrbbecDemo(const OrbbecDemo& window);
  ~OrbbecDemo();

  Ui::OrbbecDemoClass ui;
  Train* trainWindow = nullptr;
  WeightEstmation* estimationWindow = nullptr;
  QString dirpath;
};
