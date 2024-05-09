#include "MainWindow.h"

int main(int argc, char* argv[]) {
  QApplication a(argc, argv);
  a.connect(&a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()));
  OrbbecDemo* w = new OrbbecDemo;
  w->setWindowTitle(QString::fromLocal8Bit("��ũר��_������"));
  w->setAttribute(Qt::WA_DeleteOnClose);
  w->setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  w->showMaximized();
  return a.exec();
}
