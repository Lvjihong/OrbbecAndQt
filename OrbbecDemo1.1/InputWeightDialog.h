#pragma once

#include <QWidget>
#include "ui_InputDialog.h"
class InputWeightDialog : public QWidget {
  Q_OBJECT

 public:
  InputWeightDialog(QWidget* parent);
  ~InputWeightDialog();

 private:
  Ui::InputDialog ui;
};
