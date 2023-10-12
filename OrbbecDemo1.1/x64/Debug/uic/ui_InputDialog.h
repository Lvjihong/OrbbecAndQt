/********************************************************************************
** Form generated from reading UI file 'InputDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_INPUTDIALOG_H
#define UI_INPUTDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>
#include "myqlineedit.h"

QT_BEGIN_NAMESPACE

class Ui_InputDialog
{
public:
    QWidget *widget;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    MyQLineEdit *ldt_input_weight;
    QLabel *label_2;
    QPushButton *btn_cancel;
    QPushButton *btn_confirm;
    QLabel *label_3;

    void setupUi(QWidget *InputDialog)
    {
        if (InputDialog->objectName().isEmpty())
            InputDialog->setObjectName(QString::fromUtf8("InputDialog"));
        InputDialog->resize(550, 350);
        InputDialog->setMinimumSize(QSize(550, 350));
        InputDialog->setMaximumSize(QSize(550, 350));
        widget = new QWidget(InputDialog);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setGeometry(QRect(90, 130, 361, 46));
        horizontalLayout = new QHBoxLayout(widget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(widget);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout->addWidget(label);

        ldt_input_weight = new MyQLineEdit(widget);
        ldt_input_weight->setObjectName(QString::fromUtf8("ldt_input_weight"));

        horizontalLayout->addWidget(ldt_input_weight);

        label_2 = new QLabel(widget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout->addWidget(label_2);

        btn_cancel = new QPushButton(InputDialog);
        btn_cancel->setObjectName(QString::fromUtf8("btn_cancel"));
        btn_cancel->setGeometry(QRect(160, 210, 93, 28));
        btn_confirm = new QPushButton(InputDialog);
        btn_confirm->setObjectName(QString::fromUtf8("btn_confirm"));
        btn_confirm->setGeometry(QRect(280, 210, 93, 28));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(btn_confirm->sizePolicy().hasHeightForWidth());
        btn_confirm->setSizePolicy(sizePolicy);
        label_3 = new QLabel(InputDialog);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(160, 40, 221, 91));

        retranslateUi(InputDialog);

        QMetaObject::connectSlotsByName(InputDialog);
    } // setupUi

    void retranslateUi(QWidget *InputDialog)
    {
        InputDialog->setWindowTitle(QCoreApplication::translate("InputDialog", "\345\244\247\345\214\227\345\206\234\344\270\223\351\241\271_\350\276\223\345\205\245\347\224\237\347\214\252\344\275\223\351\207\215", nullptr));
        label->setText(QCoreApplication::translate("InputDialog", "<html><head/><body><p align=\"center\"><span style=\" font-size:12pt; font-weight:600;\">\350\257\267\350\276\223\345\205\245\347\224\237\347\214\252\344\275\223\351\207\215\357\274\232</span></p></body></html>", nullptr));
        label_2->setText(QCoreApplication::translate("InputDialog", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">Kg</span></p></body></html>", nullptr));
        btn_cancel->setText(QCoreApplication::translate("InputDialog", "\345\217\226\346\266\210", nullptr));
        btn_confirm->setText(QCoreApplication::translate("InputDialog", "\347\241\256\350\256\244", nullptr));
        label_3->setText(QCoreApplication::translate("InputDialog", "<html><head/><body><p align=\"center\"><span style=\" font-size:18pt; font-weight:600;\">\351\207\207\351\233\206\345\256\214\346\210\220\357\274\201</span></p></body></html>", nullptr));
    } // retranslateUi

};

namespace Ui {
    class InputDialog: public Ui_InputDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_INPUTDIALOG_H
