#include "MainWindow.h"
#include <qpushbutton.h>
#include <QFileDialog>
#include "train.h"
OrbbecDemo::OrbbecDemo(QWidget* parent) : QMainWindow(parent) {
  ui.setupUi(this);
  QFont font("����", 35);
  ui.btn_train->setFont(font);
  ui.btn_guess->setFont(font);
  connect(ui.btn_train, &QPushButton::clicked, [=]() mutable {
    QString dirpath = QFileDialog::getExistingDirectory(
        this, QString::fromLocal8Bit("��ũר��_ѡ�����ݴ洢λ��"), "./",
        QFileDialog::ShowDirsOnly);
    QDir dir(dirpath);
    if (!dir.exists()) {
      dir.mkdir(dirpath);
    }
    Train* newWindow = nullptr;
    if (newWindow == nullptr) {
      newWindow = new Train(dirpath, this);
      newWindow->setWindowTitle(
          QString::fromLocal8Bit("��ũר��_ѵ�������ռ�"));
      newWindow->showMaximized();
    } else {
      newWindow->showMaximized();
    }
  });
}

OrbbecDemo::~OrbbecDemo() {}
