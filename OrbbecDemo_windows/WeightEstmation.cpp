#include "WeightEstmation.h"
#include "qdebug.h"
WeightEstmation::WeightEstmation(const QString dirPath, QWidget* parent)
    : QWidget(parent) {
  ui.setupUi(this);
  qDebug() << dirPath << "------------------------------";

}

WeightEstmation::~WeightEstmation()
{}
