#include "InputWeightDialog.h"

const double EPS = 1e-6;
QString number[12] = {"1", "2", "3", "4", "5", "6", "7", "8", "9", ".", "0"};
InputWeightDialog::InputWeightDialog(QString dirpath, QWidget* parent)
    : QWidget(parent) {
  ui.setupUi(this);

  connect(ui.buttonGroup, SIGNAL(buttonClicked(int)),
          SLOT(buttonClickResponse(int)));
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
InputWeightDialog::InputWeightDialog(const InputWeightDialog& dialog) {
  ui = dialog.ui;
  event = new QKeyEvent(*dialog.event);
}
InputWeightDialog::~InputWeightDialog() {
  if (event != nullptr) {
    delete event;
    event = nullptr;
  }
}

void InputWeightDialog::closeEvent(QCloseEvent* event) {
  if (ui.ldt_input_weight->text() == "") {
    QErrorMessage* dialog = new QErrorMessage(this);
    dialog->setWindowTitle(QString::fromLocal8Bit("体重错误"));
    dialog->showMessage(QString::fromLocal8Bit("体重不能为空！"));
    event->ignore();
  }
}

void InputWeightDialog::buttonClickResponse(int key) {
  if (key == -11) {  // 小数点
    event =
        new QKeyEvent(QEvent::KeyPress, 0, Qt::NoModifier, number[-key - 2]);
    ui.pushButton_10->setDisabled(true);
  } else if (key == -13) {  // backspace
    event = new QKeyEvent(QEvent::KeyPress, Qt::Key_Backspace,
                          Qt::NoModifier);  //新建一个键盘事件
    ui.ldt_input_weight->setFocus();
    QString number = ui.ldt_input_weight->text();
    if (number.at(number.size() - 1) == ".") {
      ui.pushButton_10->setDisabled(false);
    }
  } else {
    event =
        new QKeyEvent(QEvent::KeyPress, 0, Qt::NoModifier, number[-key - 2]);
  }
  ui.ldt_input_weight->setFocus();
  QApplication::sendEvent(focusWidget(), event);
}