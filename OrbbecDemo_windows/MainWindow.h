#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"
#include <qpushbutton.h>
#include <QFileDialog>
#include "Train.h"
#include "WeightEstmation.h"

#ifdef _DEBUG
#pragma comment(lib, "opencv_world480d.lib")
#else
#pragma comment(lib, "opencv_world480.lib")
#endif
class OrbbecDemo : public QMainWindow
{
    Q_OBJECT

public:
    OrbbecDemo(QWidget* parent = nullptr);
    OrbbecDemo(const OrbbecDemo&window);
    ~OrbbecDemo();

    Ui::OrbbecDemoClass ui;
    Train* trainWindow = nullptr;
    WeightEstmation* estimationWindow = nullptr;
    QString dirpath;
};
