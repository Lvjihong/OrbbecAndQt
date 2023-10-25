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

    if (trainWindow == nullptr) {
      trainWindow = new Train(dirpath, this);
      trainWindow->setWindowFlags(Qt::Window);
      trainWindow->showMaximized();
    } else {
      delete trainWindow;
      trainWindow = new Train(dirpath, this);
      trainWindow->setWindowFlags(Qt::Window);
      trainWindow->showMaximized();
    }
  });
  connect(ui.btn_estimation, &QPushButton::clicked, [=]() mutable {
    dirpath = QFileDialog::getExistingDirectory(
        this, QString::fromLocal8Bit("大北农专项_选择需要估计的数据"), "./",
        QFileDialog::ShowDirsOnly);

    QDir dir(dirpath);
    if (!dir.exists()) {
      dir.mkdir(dirpath);
    }

    if (estimationWindow == nullptr) {
      estimationWindow = new WeightEstmation(dirpath, this);
      estimationWindow->setWindowFlags(Qt::Window);
      estimationWindow->showMaximized();
    } else {
      delete estimationWindow;
      estimationWindow = new WeightEstmation(dirpath, this);
      estimationWindow->setWindowFlags(Qt::Window);
      estimationWindow->showMaximized();
    }
  });
}

OrbbecDemo::~OrbbecDemo() {}
