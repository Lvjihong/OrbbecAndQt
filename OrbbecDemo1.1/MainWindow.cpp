#include "MainWindow.h"
#include <qdialog.h>
#include <qpushbutton.h>
#include "train.h"
OrbbecDemo::OrbbecDemo(QWidget* parent) : QMainWindow(parent) {
  ui.setupUi(this);
  Train* newWindow = nullptr;
  connect(ui.btn_train, &QPushButton::clicked, [=]() mutable {
    if (newWindow == nullptr) {
      newWindow = new Train(this);
      newWindow->showMaximized();
    } else {
      newWindow->showMaximized();
    }
  });
}

OrbbecDemo::~OrbbecDemo() {}
