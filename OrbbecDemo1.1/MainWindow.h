#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"

class OrbbecDemo : public QMainWindow
{
    Q_OBJECT

public:
    OrbbecDemo(QWidget *parent = nullptr);
    ~OrbbecDemo();

private:
    Ui::OrbbecDemoClass ui;
};
