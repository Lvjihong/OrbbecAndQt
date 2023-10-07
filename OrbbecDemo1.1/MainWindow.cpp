#include "MainWindow.h"
#include <qpushbutton.h>
#include "InputWeightDialog.h"

OrbbecDemo::OrbbecDemo(QWidget* parent) : QMainWindow(parent) {
  ui.setupUi(this);
  connect(ui.btn_train, &QPushButton::clicked, [=]() mutable {
    InputWeightDialog* dialog = new InputWeightDialog(this);
    dialog->setWindowFlags(Qt::Window);
    dialog->setWindowModality(Qt::WindowModal);
    dialog->show();
  });
}

OrbbecDemo::~OrbbecDemo() {}
