#include <QtWidgets/QApplication>
#include "MainWindow.h"

int main(int argc, char* argv[]) {
  // ϵͳ�Դ�����̣���Ч
  // qputenv("QT_IM_MODULE", QByteArray("qtvirtualkeyboard"));
  QApplication a(argc, argv);
  a.connect(&a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()));
  OrbbecDemo* w = new OrbbecDemo;
  w->setWindowTitle(QString::fromLocal8Bit("��ũר��_������"));
  w->setAttribute(Qt::WA_DeleteOnClose);
  w->show();
  return a.exec();
}
