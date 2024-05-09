#pragma once

#include "ui_InputDialog.h"
#include <QCloseEvent>
#include <QErrorMessage>
#include <QFileDialog>
#include <QWidget>
#include <QKeyEvent>
#include <QFile>
#include <QTextStream>

class InputWeightDialog : public QWidget {
  Q_OBJECT

 public:
  InputWeightDialog(QString dirpath, QWidget* parent);
  InputWeightDialog(const InputWeightDialog& dialog);
  ~InputWeightDialog();

 public slots:
  void buttonClickResponse(int key);

  void getHorValue(int value);

 signals:
  void sendMessage(QString gemfield);
  void inputOver();

 protected:
  void closeEvent(QCloseEvent* event);

 private:
  Ui::InputDialog ui;
  QKeyEvent* event;
};
