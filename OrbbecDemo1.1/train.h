
#include <opencv2/opencv.hpp>
#include "libobsensor/hpp/Pipeline.hpp"
#include "ui_Train.h"
#include <QCloseEvent>
class Train : public QWidget {
  Q_OBJECT

 public:
  Train(const QString dirPath, QWidget* parent = nullptr);
  ~Train();
  void saveOrShowAll(bool flag, QString rootDirPath);
  void closeEvent(QCloseEvent* e);
  void startRecord();
  void stopRecord();

 private:
  Ui::TrainWindow ui;

  ob::Pipeline pipe;
  std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
  int depthCount;
  int colorCount;
  QString rootDirPath;
  bool isSave = false;
};
