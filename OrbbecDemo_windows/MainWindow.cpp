#include "MainWindow.h"
OrbbecDemo::OrbbecDemo(QWidget* parent) : QMainWindow(parent) {
  ui.setupUi(this);
  QFont font("����", 35);
  ui.btn_train->setFont(font);
  ui.btn_guess->setFont(font);
  connect(ui.btn_train, &QPushButton::clicked, [=]() mutable {
    dirpath = QFileDialog::getExistingDirectory(
        this, QString::fromLocal8Bit("��ũר��_ѡ�����ݴ洢λ��"), "./",
        QFileDialog::ShowDirsOnly);
    QDir dir(dirpath);
    if (!dir.exists()) {
      dir.mkdir(dirpath);
    }

    if (newWindow == nullptr) {
      newWindow = new Train(dirpath, this);
      newWindow->setWindowFlags(Qt::Window);
      newWindow->setWindowTitle(
          QString::fromLocal8Bit("��ũר��_ѵ�������ռ�"));
      newWindow->showMaximized();
    } else {
      delete newWindow;
      newWindow = new Train(dirpath, this);
      newWindow->setWindowFlags(Qt::Window);
      newWindow->setWindowTitle(
          QString::fromLocal8Bit("��ũר��_ѵ�������ռ�"));
      newWindow->showMaximized();
    }
  });
}

OrbbecDemo::~OrbbecDemo() {}
