#pragma once
#include <QWidget>
#include "ui_WeightEstimation.h"
class WeightEstmation : public QWidget {
  Q_OBJECT

 public:
  WeightEstmation(const QString dirPath, QWidget* parent = nullptr);
  WeightEstmation(const WeightEstmation& window);
  ~WeightEstmation();

 private:
  Ui::WeightEstimation ui;
};
