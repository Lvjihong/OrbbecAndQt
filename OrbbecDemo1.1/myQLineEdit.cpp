#include "myQLineEdit.h"

MyQLineEdit::MyQLineEdit(QWidget *parent)
	: QLineEdit(parent)
{}

MyQLineEdit::~MyQLineEdit()
{}

void MyQLineEdit::mousePressEvent(QMouseEvent* event) {
  QLineEdit::mousePressEvent(event);
  setFocus();
  QInputMethod* inputMethod =
      QGuiApplication::inputMethod();
  inputMethod->show();

}
