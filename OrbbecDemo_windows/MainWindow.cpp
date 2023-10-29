#include "MainWindow.h"
OrbbecDemo::OrbbecDemo(QWidget* parent) : QMainWindow(parent) {
  ui.setupUi(this);

  connect(ui.btn_train, &QPushButton::clicked, [=]() mutable {
    dirpath = QFileDialog::getExistingDirectory(
        this, QString::fromLocal8Bit("大北农专项_选择数据存储位置"), "./",
        QFileDialog::ShowDirsOnly);

    QDir dir(dirpath);
    if (!dir.exists()) {
      dir.mkdir(dirpath);
    }
    if (trainWindow != nullptr) {
      delete trainWindow;
      trainWindow = nullptr;
    }
    trainWindow = new Train(dirpath, this);
    trainWindow->setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
    trainWindow->showMaximized();
    connect(this, &OrbbecDemo::windowShowed, trainWindow,
            &Train::saveOrShowAll);
    emit windowShowed(false, QString());
  });
  connect(ui.btn_estimation, &QPushButton::clicked, [=]() mutable {
    dirpath = QFileDialog::getExistingDirectory(
        this, QString::fromLocal8Bit("大北农专项_选择需要估计的数据"), "./",
        QFileDialog::ShowDirsOnly);

    QDir dir(dirpath);
    if (!dir.exists()) {
      dir.mkdir(dirpath);
    }
    if (estimationWindow != nullptr) {
      delete estimationWindow;
      estimationWindow = nullptr;
    }
    estimationWindow = new WeightEstmation(dirpath, this);
    estimationWindow->setWindowFlags(Qt::Window);
    estimationWindow->showMaximized();
  });
}
OrbbecDemo::OrbbecDemo(const OrbbecDemo& window) {
  ui = window.ui;
  trainWindow = new Train(*window.trainWindow);
  estimationWindow = new WeightEstmation(*window.estimationWindow);
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
