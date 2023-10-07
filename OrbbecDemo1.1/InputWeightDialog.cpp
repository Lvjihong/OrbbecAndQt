#include "InputWeightDialog.h"
#include <QFileDialog>
#include <QTextStream>
#include "train.h"
InputWeightDialog::InputWeightDialog(QWidget* parent) : QWidget(parent) {
  ui.setupUi(this);
  QString dirpath = QFileDialog::getExistingDirectory(
      this, QString::fromLocal8Bit("��ũר��_ѡ�����ݴ洢λ��"), "./",
      QFileDialog::ShowDirsOnly);
  QDir dir(dirpath);
  if (!dir.exists()) {
    dir.mkdir(dirpath);
  }
  if (!dir.exists("Depth")) {
    dir.mkdir("Depth");
  }
  if (!dir.exists("Rgb")) {
    dir.mkdir("Rgb");
  }
  connect(ui.btn_confirm, &QPushButton::clicked, [=]() {
    // bool exist = QFile::exists(dirpath + "/weight.txt");
    QFile* file = new QFile(dirpath + "/weight.txt");
    bool ok = file->open(QIODevice::ReadWrite | QIODevice::Text);
    QString weight = ui.ldt_input_weight->text();
    file->write(weight.toUtf8());
    file->close();
    delete file;
    this->close();

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

InputWeightDialog::~InputWeightDialog() {}
