#include <QCloseEvent>
#include <QErrorMessage>
#include <QFileDialog>
#include <QWidget>
#include "ui_InputDialog.h"
class InputWeightDialog : public QWidget {
  Q_OBJECT

 public:
  InputWeightDialog(QString dirpath, QWidget* parent);
  ~InputWeightDialog();


 signals:
  void inputOver();

 protected:
  void closeEvent(QCloseEvent* event);

 private:
  Ui::InputDialog ui;
};
