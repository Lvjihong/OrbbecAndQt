#include <opencv2/opencv.hpp>
#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Frame.hpp>
#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/Error.hpp"
#include "libobsensor/hpp/StreamProfile.hpp"
#include "ui_Train.h"
#include <qdirmodel.h>
#include <thread>
#include "InputWeightDialog.h"
#include <QFileDialog>
#include <QCloseEvent>
#include <qpushbutton.h>
class Train : public QWidget {
  Q_OBJECT

 public:
  Train(const QString dirPath, QWidget* parent = nullptr);
  ~Train();
  void closeEvent(QCloseEvent* e);
  void saveOrShowAll(bool flag, QString rootDirPath);

 private slots:
  void updateTreeView();

 private:
  Ui::TrainWindow ui;

  ob::Pipeline pipe;
  std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
  int depthCount;
  int colorCount;
  QString rootDirPath;
  bool isSave = false;
  QDirModel* model;
};
