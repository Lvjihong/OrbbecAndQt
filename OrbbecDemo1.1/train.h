
#include <opencv2/opencv.hpp>
#include "libobsensor/hpp/Pipeline.hpp"
#include "ui_Train.h"
class Train : public QWidget {
  Q_OBJECT

 public:
  Train(QWidget* parent = nullptr);
  ~Train();
  void saveOrShowAll(bool flag);
  void startRecord();
  void stopRecord();

 private:
  Ui::Form ui;

  ob::Pipeline pipe;
  std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
  int depthCount;
  int colorCount;

  bool isSave = false;
};
