#include "MainWindow.h"
#include <qpushbutton.h>
#include <QFileDialog>
#include "train.h"
OrbbecDemo::OrbbecDemo(QWidget* parent) : QMainWindow(parent) {
  ui.setupUi(this);
  QFont font("黑体", 35);
  ui.btn_train->setFont(font);
  ui.btn_guess->setFont(font);
  connect(ui.btn_train, &QPushButton::clicked, [=]() mutable {
    QString dirpath = QFileDialog::getExistingDirectory(
        this, QString::fromLocal8Bit("大北农专项_选择数据存储位置"), "./",
        QFileDialog::ShowDirsOnly);
    QDir dir(dirpath);
    if (!dir.exists()) {
      dir.mkdir(dirpath);
    }
    Train* newWindow = nullptr;
    if (newWindow == nullptr) {
      newWindow = new Train(dirpath, this);
      newWindow->setWindowTitle(
          QString::fromLocal8Bit("大北农专项_训练数据收集"));
      newWindow->showMaximized();
    } else {
      newWindow->showMaximized();
    }
  });
}

OrbbecDemo::~OrbbecDemo() {}
