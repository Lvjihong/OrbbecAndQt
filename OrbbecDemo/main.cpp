#include <QtWidgets/QApplication>
#include "MainWindow.h"

int main(int argc, char* argv[]) {
  QApplication a(argc, argv);
  a.connect(&a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()));
  OrbbecDemo* w = new OrbbecDemo;
  w->setAttribute(Qt::WA_DeleteOnClose);
  w->show();
  return a.exec();
}
