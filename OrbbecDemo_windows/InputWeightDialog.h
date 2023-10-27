#include <QCloseEvent>
#include <QErrorMessage>
#include <QFileDialog>
#include <QWidget>
#include "ui_InputDialog.h"

#include <QKeyEvent>

class InputWeightDialog : public QWidget {
  Q_OBJECT

 public:
  InputWeightDialog(QString dirpath, QWidget* parent);
  InputWeightDialog(const InputWeightDialog& dialog);
  ~InputWeightDialog();
  
public slots:
  void buttonClickResponse(int key);
 signals:
  void sendMessage(QString gemfield);
  void inputOver();

 protected:
  void closeEvent(QCloseEvent* event);

 private:
  Ui::InputDialog ui;

  QKeyEvent* event;
};
