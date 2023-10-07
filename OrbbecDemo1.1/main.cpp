#include "MainWindow.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  a.connect(&a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()));
  OrbbecDemo* w = new OrbbecDemo;
  w->setWindowTitle(QString::fromLocal8Bit("大北农专项_主界面"));
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  w->setAttribute(Qt::WA_DeleteOnClose);
  w->show();
  return a.exec();
}
