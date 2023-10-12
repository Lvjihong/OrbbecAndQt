#include "InputWeightDialog.h"

const double EPS = 1e-6;
InputWeightDialog::InputWeightDialog(QString dirpath, QWidget* parent)
    : QWidget(parent) {
  ui.setupUi(this);


  connect(ui.btn_confirm, &QPushButton::clicked, [=]() {
    QFile* file = new QFile(dirpath + "/weight.txt");
    bool ok = file->open(QIODevice::ReadWrite | QIODevice::Text);
    QString weight = ui.ldt_input_weight->text();
    double weight_d = weight.toDouble();

    if (weight_d > 0 + EPS && weight_d < 300 + EPS) {
      file->write(weight.toUtf8());
      file->close();
      delete file;
      emit inputOver();
      this->close();
    } else {
      file->close();
      QErrorMessage* dialog = new QErrorMessage(this);
      dialog->setWindowTitle(QString::fromLocal8Bit("体重错误"));
      dialog->showMessage(QString::fromLocal8Bit("请输入合适的体重！"));
    }
  });

  connect(ui.btn_cancel, &QPushButton::clicked, [=]() {
    QErrorMessage* dialog = new QErrorMessage(this);
    dialog->setWindowTitle(QString::fromLocal8Bit("体重错误"));
    dialog->showMessage(QString::fromLocal8Bit("体重不能为空！"));
  });
}

InputWeightDialog::~InputWeightDialog() {}

void InputWeightDialog::closeEvent(QCloseEvent* event) {
  if (ui.ldt_input_weight->text() == "") {
    QErrorMessage* dialog = new QErrorMessage(this);
    dialog->setWindowTitle(QString::fromLocal8Bit("体重错误"));
    dialog->showMessage(QString::fromLocal8Bit("体重不能为空！"));
    event->ignore();
  }
}

