#include <QWidget>
#include "ui_InputDialog.h"
#include <QCloseEvent> 
class InputWeightDialog : public QWidget {
  Q_OBJECT

 public:
  InputWeightDialog(QString dirpath, QWidget* parent);
  ~InputWeightDialog();

 protected:


 private:
  Ui::InputDialog ui;
};
