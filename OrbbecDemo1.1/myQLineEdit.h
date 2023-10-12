#pragma once
#include <QApplication>
#include <QInputMethod>
#include <QLineEdit>
#include <QMouseEvent>

class MyQLineEdit : public QLineEdit {
  Q_OBJECT

 protected:
  void mousePressEvent(QMouseEvent* event);

 public:
  MyQLineEdit(QWidget* parent);
  ~MyQLineEdit();
};
