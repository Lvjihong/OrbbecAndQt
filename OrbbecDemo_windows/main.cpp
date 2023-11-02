#include <QtWidgets/QApplication>
#include "MainWindow.h"
#undef slots
#include "torch/script.h"
#define slots Q_SLOTS

int main(int argc, char* argv[]) {
  QApplication a(argc, argv);
  a.connect(&a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()));
  OrbbecDemo* w = new OrbbecDemo;
  w->setWindowTitle(QString::fromLocal8Bit("大北农专项_主界面"));
  w->setAttribute(Qt::WA_DeleteOnClose);
  w->setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  w->showMaximized();
  return a.exec();
}
