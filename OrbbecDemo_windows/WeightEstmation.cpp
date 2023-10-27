#include "WeightEstmation.h"

WeightEstmation::WeightEstmation(const QString dirPath, QWidget* parent)
    : QWidget(parent) {
  ui.setupUi(this);

}

WeightEstmation::WeightEstmation(const WeightEstmation& window){
  ui = window.ui;

}

WeightEstmation::~WeightEstmation()
{}
