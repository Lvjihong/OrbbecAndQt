#ifdef _DEBUG
#pragma comment(lib, "opencv_world480d.lib")
#else
#pragma comment(lib, "opencv_world480.lib")
#endif
#include <qpushbutton.h>

#include <QFileDialog>
#include <QtWidgets/QMainWindow>

#include "Train.h"
#include "WeightEstmation.h"
#include "ui_MainWindow.h"

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
  WeightEstmation* estimationWindow = nullptr;
  QString dirpath;
};
