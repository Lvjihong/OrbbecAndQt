#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"
#include <qpushbutton.h>
#include <QFileDialog>
#include "train.h"

#ifdef _DEBUG
#pragma comment(lib, "opencv_world480d.lib")
#else
#pragma comment(lib, "opencv_world480.lib")
#endif
class OrbbecDemo : public QMainWindow
{
    Q_OBJECT

public:
    OrbbecDemo(QWidget *parent = nullptr);
    ~OrbbecDemo();

    Ui::OrbbecDemoClass ui;
    Train* newWindow = nullptr;
    QString dirpath;
};
