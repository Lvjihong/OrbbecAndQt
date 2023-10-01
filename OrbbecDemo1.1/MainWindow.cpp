#include "MainWindow.h"
#include <qdialog.h>
#include <qpushbutton.h>
#include "train.h"
OrbbecDemo::OrbbecDemo(QWidget* parent) : QMainWindow(parent) {
  ui.setupUi(this);
  connect(ui.btn_train, &QPushButton::clicked, [=]() {
    Train* newWindow = new Train(this);
    newWindow->showMaximized();
    newWindow->setAttribute(Qt::WA_DeleteOnClose);
  });
}

OrbbecDemo::~OrbbecDemo() {}
