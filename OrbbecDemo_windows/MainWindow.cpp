#include "MainWindow.h"
OrbbecDemo::OrbbecDemo(QWidget* parent) : QMainWindow(parent) {
  ui.setupUi(this);
  connect(ui.btn_exit, &QPushButton::clicked, this, &OrbbecDemo::close);
  connect(ui.btn_train, &QPushButton::clicked, [=]() mutable {
    dirpath = QFileDialog::getExistingDirectory(
        this, QString::fromLocal8Bit("大北农专项_选择数据存储位置"), "./",
        QFileDialog::ShowDirsOnly);
    if (dirpath.isEmpty()) return;
    QDir dir(dirpath);
    if (!dir.exists()) {
      dir.mkdir(dirpath);
    }
    if (trainWindow != nullptr) {
      delete trainWindow;
      trainWindow = nullptr;
    }
    if (estimationWindow != nullptr) {
      delete estimationWindow;
      estimationWindow = nullptr;
    }
    trainWindow = new Train(dirpath, this);
    trainWindow->setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
    trainWindow->showMaximized();
    connect(this, &OrbbecDemo::trainWindowShowed, trainWindow,
            &Train::saveOrShowAll);
    emit trainWindowShowed(false, QString());
  });
  connect(ui.btn_estimation, &QPushButton::clicked, [=]() mutable {
    if (trainWindow != nullptr) {
      delete trainWindow;
      trainWindow = nullptr;
    }
    if (estimationWindow != nullptr) {
      delete estimationWindow;
      estimationWindow = nullptr;
    }
    estimationWindow = new WeightEstimation(this);
    estimationWindow->setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
    estimationWindow->showMaximized();

    //connect(this, &OrbbecDemo::estimationWindowShowed, estimationWindow,
    //        &WeightEstimation::showDepth);
    //emit estimationWindowShowed();
  });
}
OrbbecDemo::OrbbecDemo(const OrbbecDemo& window) {
  ui = window.ui;
  trainWindow = new Train(*window.trainWindow);
  estimationWindow = new WeightEstimation(*window.estimationWindow);
  dirpath = window.dirpath;
}
OrbbecDemo::~OrbbecDemo() {
  if (trainWindow != nullptr) {
    delete trainWindow;
    trainWindow = nullptr;
  }
  if (estimationWindow != nullptr) {
    delete estimationWindow;
    estimationWindow = nullptr;
  }
}