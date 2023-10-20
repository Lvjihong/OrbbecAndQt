#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"
#include <opencv2/opencv.hpp>
#include "libobsensor/hpp/Pipeline.hpp"
#include <windows.h> 
class OrbbecDemo : public QMainWindow
{
    Q_OBJECT

public:
    OrbbecDemo(QWidget *parent = nullptr);
 ~OrbbecDemo();
    void showDepth(bool isRecord = false);
    void showRGB(bool isRecord = false);
    void startDepth();
    void stopDepth();
    void startRGB();
    void stopRGB();
    void saveOrShowAll();

   private:
    Ui::OrbbecDemoClass ui;

    ob::Pipeline pipe;
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    int depthCount;
    int colorCount;

};
