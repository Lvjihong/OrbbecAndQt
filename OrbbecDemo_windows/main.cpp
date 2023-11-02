#include <QtWidgets/QApplication>
#include "MainWindow.h"
#undef slots
#include "torch/script.h"
#define slots Q_SLOTS

int main(int argc, char* argv[]) {
  // 系统自带软键盘，无效
  // qputenv("QT_IM_MODULE", QByteArray("qtvirtualkeyboard"));
  QApplication a(argc, argv);
  a.connect(&a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()));
  OrbbecDemo* w = new OrbbecDemo;
  w->setWindowTitle(QString::fromLocal8Bit("大北农专项_主界面"));
  w->setAttribute(Qt::WA_DeleteOnClose);
  w->showMaximized();
  return a.exec();
}
