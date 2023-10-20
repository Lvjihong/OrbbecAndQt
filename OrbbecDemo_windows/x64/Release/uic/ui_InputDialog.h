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
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_InputDialog
{
public:
    QWidget *widget;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLineEdit *ldt_input_weight;
    QLabel *label_2;
    QPushButton *btn_cancel;
    QPushButton *btn_confirm;
    QLabel *label_3;
    QWidget *widget_2;
    QGridLayout *gridLayout;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QPushButton *pushButton_3;
    QPushButton *pushButton_4;
    QPushButton *pushButton_5;
    QPushButton *pushButton_6;
    QPushButton *pushButton_7;
    QPushButton *pushButton_8;
    QPushButton *pushButton_9;
    QPushButton *pushButton_10;
    QPushButton *pushButton_11;
    QPushButton *pushButton_12;
    QButtonGroup *buttonGroup;

    void setupUi(QWidget *InputDialog)
    {
        if (InputDialog->objectName().isEmpty())
            InputDialog->setObjectName(QString::fromUtf8("InputDialog"));
        InputDialog->resize(750, 550);
        InputDialog->setMinimumSize(QSize(750, 550));
        InputDialog->setMaximumSize(QSize(750, 550));
        widget = new QWidget(InputDialog);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setGeometry(QRect(190, 110, 361, 46));
        horizontalLayout = new QHBoxLayout(widget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(widget);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout->addWidget(label);

        ldt_input_weight = new QLineEdit(widget);
        ldt_input_weight->setObjectName(QString::fromUtf8("ldt_input_weight"));

        horizontalLayout->addWidget(ldt_input_weight);

        label_2 = new QLabel(widget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout->addWidget(label_2);

        btn_cancel = new QPushButton(InputDialog);
        btn_cancel->setObjectName(QString::fromUtf8("btn_cancel"));
        btn_cancel->setGeometry(QRect(180, 460, 150, 50));
        btn_confirm = new QPushButton(InputDialog);
        btn_confirm->setObjectName(QString::fromUtf8("btn_confirm"));
        btn_confirm->setGeometry(QRect(400, 460, 150, 50));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(btn_confirm->sizePolicy().hasHeightForWidth());
        btn_confirm->setSizePolicy(sizePolicy);
        label_3 = new QLabel(InputDialog);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(250, 30, 251, 71));
        widget_2 = new QWidget(InputDialog);
        widget_2->setObjectName(QString::fromUtf8("widget_2"));
        widget_2->setGeometry(QRect(220, 180, 300, 250));
        gridLayout = new QGridLayout(widget_2);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        pushButton = new QPushButton(widget_2);
        buttonGroup = new QButtonGroup(InputDialog);
        buttonGroup->setObjectName(QString::fromUtf8("buttonGroup"));
        buttonGroup->addButton(pushButton);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        QSizePolicy sizePolicy1(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(pushButton->sizePolicy().hasHeightForWidth());
        pushButton->setSizePolicy(sizePolicy1);

        gridLayout->addWidget(pushButton, 0, 0, 1, 1);

        pushButton_2 = new QPushButton(widget_2);
        buttonGroup->addButton(pushButton_2);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        sizePolicy1.setHeightForWidth(pushButton_2->sizePolicy().hasHeightForWidth());
        pushButton_2->setSizePolicy(sizePolicy1);

        gridLayout->addWidget(pushButton_2, 0, 1, 1, 1);

        pushButton_3 = new QPushButton(widget_2);
        buttonGroup->addButton(pushButton_3);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));
        sizePolicy1.setHeightForWidth(pushButton_3->sizePolicy().hasHeightForWidth());
        pushButton_3->setSizePolicy(sizePolicy1);

        gridLayout->addWidget(pushButton_3, 0, 2, 1, 1);

        pushButton_4 = new QPushButton(widget_2);
        buttonGroup->addButton(pushButton_4);
        pushButton_4->setObjectName(QString::fromUtf8("pushButton_4"));
        sizePolicy1.setHeightForWidth(pushButton_4->sizePolicy().hasHeightForWidth());
        pushButton_4->setSizePolicy(sizePolicy1);

        gridLayout->addWidget(pushButton_4, 1, 0, 1, 1);

        pushButton_5 = new QPushButton(widget_2);
        buttonGroup->addButton(pushButton_5);
        pushButton_5->setObjectName(QString::fromUtf8("pushButton_5"));
        sizePolicy1.setHeightForWidth(pushButton_5->sizePolicy().hasHeightForWidth());
        pushButton_5->setSizePolicy(sizePolicy1);

        gridLayout->addWidget(pushButton_5, 1, 1, 1, 1);

        pushButton_6 = new QPushButton(widget_2);
        buttonGroup->addButton(pushButton_6);
        pushButton_6->setObjectName(QString::fromUtf8("pushButton_6"));
        sizePolicy1.setHeightForWidth(pushButton_6->sizePolicy().hasHeightForWidth());
        pushButton_6->setSizePolicy(sizePolicy1);

        gridLayout->addWidget(pushButton_6, 1, 2, 1, 1);

        pushButton_7 = new QPushButton(widget_2);
        buttonGroup->addButton(pushButton_7);
        pushButton_7->setObjectName(QString::fromUtf8("pushButton_7"));
        sizePolicy1.setHeightForWidth(pushButton_7->sizePolicy().hasHeightForWidth());
        pushButton_7->setSizePolicy(sizePolicy1);

        gridLayout->addWidget(pushButton_7, 2, 0, 1, 1);

        pushButton_8 = new QPushButton(widget_2);
        buttonGroup->addButton(pushButton_8);
        pushButton_8->setObjectName(QString::fromUtf8("pushButton_8"));
        sizePolicy1.setHeightForWidth(pushButton_8->sizePolicy().hasHeightForWidth());
        pushButton_8->setSizePolicy(sizePolicy1);

        gridLayout->addWidget(pushButton_8, 2, 1, 1, 1);

        pushButton_9 = new QPushButton(widget_2);
        buttonGroup->addButton(pushButton_9);
        pushButton_9->setObjectName(QString::fromUtf8("pushButton_9"));
        sizePolicy1.setHeightForWidth(pushButton_9->sizePolicy().hasHeightForWidth());
        pushButton_9->setSizePolicy(sizePolicy1);

        gridLayout->addWidget(pushButton_9, 2, 2, 1, 1);

        pushButton_10 = new QPushButton(widget_2);
        buttonGroup->addButton(pushButton_10);
        pushButton_10->setObjectName(QString::fromUtf8("pushButton_10"));
        sizePolicy1.setHeightForWidth(pushButton_10->sizePolicy().hasHeightForWidth());
        pushButton_10->setSizePolicy(sizePolicy1);

        gridLayout->addWidget(pushButton_10, 3, 0, 1, 1);

        pushButton_11 = new QPushButton(widget_2);
        buttonGroup->addButton(pushButton_11);
        pushButton_11->setObjectName(QString::fromUtf8("pushButton_11"));
        sizePolicy1.setHeightForWidth(pushButton_11->sizePolicy().hasHeightForWidth());
        pushButton_11->setSizePolicy(sizePolicy1);

        gridLayout->addWidget(pushButton_11, 3, 1, 1, 1);

        pushButton_12 = new QPushButton(widget_2);
        buttonGroup->addButton(pushButton_12);
        pushButton_12->setObjectName(QString::fromUtf8("pushButton_12"));
        sizePolicy1.setHeightForWidth(pushButton_12->sizePolicy().hasHeightForWidth());
        pushButton_12->setSizePolicy(sizePolicy1);

        gridLayout->addWidget(pushButton_12, 3, 2, 1, 1);


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
        pushButton->setText(QCoreApplication::translate("InputDialog", "1", nullptr));
        pushButton_2->setText(QCoreApplication::translate("InputDialog", "2", nullptr));
        pushButton_3->setText(QCoreApplication::translate("InputDialog", "3", nullptr));
        pushButton_4->setText(QCoreApplication::translate("InputDialog", "4", nullptr));
        pushButton_5->setText(QCoreApplication::translate("InputDialog", "5", nullptr));
        pushButton_6->setText(QCoreApplication::translate("InputDialog", "6", nullptr));
        pushButton_7->setText(QCoreApplication::translate("InputDialog", "7", nullptr));
        pushButton_8->setText(QCoreApplication::translate("InputDialog", "8", nullptr));
        pushButton_9->setText(QCoreApplication::translate("InputDialog", "9", nullptr));
        pushButton_10->setText(QCoreApplication::translate("InputDialog", ".", nullptr));
        pushButton_11->setText(QCoreApplication::translate("InputDialog", "0", nullptr));
        pushButton_12->setText(QCoreApplication::translate("InputDialog", "del", nullptr));
    } // retranslateUi

};

namespace Ui {
    class InputDialog: public Ui_InputDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_INPUTDIALOG_H
