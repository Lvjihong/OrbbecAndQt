/********************************************************************************
** Form generated from reading UI file 'Train.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TRAIN_H
#define UI_TRAIN_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Form
{
public:
    QGridLayout *gridLayout;
    QLabel *label_rgb;
    QPushButton *btn_start;
    QLabel *label_depth;
    QPushButton *btn_open;

    void setupUi(QWidget *Form)
    {
        if (Form->objectName().isEmpty())
            Form->setObjectName(QString::fromUtf8("Form"));
        Form->resize(1072, 547);
        Form->setMinimumSize(QSize(1072, 547));
        gridLayout = new QGridLayout(Form);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label_rgb = new QLabel(Form);
        label_rgb->setObjectName(QString::fromUtf8("label_rgb"));
        label_rgb->setMinimumSize(QSize(72, 15));

        gridLayout->addWidget(label_rgb, 5, 1, 1, 1, Qt::AlignHCenter|Qt::AlignVCenter);

        btn_start = new QPushButton(Form);
        btn_start->setObjectName(QString::fromUtf8("btn_start"));

        gridLayout->addWidget(btn_start, 3, 0, 1, 2);

        label_depth = new QLabel(Form);
        label_depth->setObjectName(QString::fromUtf8("label_depth"));
        label_depth->setMinimumSize(QSize(72, 15));

        gridLayout->addWidget(label_depth, 5, 0, 1, 1, Qt::AlignHCenter|Qt::AlignVCenter);

        btn_open = new QPushButton(Form);
        btn_open->setObjectName(QString::fromUtf8("btn_open"));

        gridLayout->addWidget(btn_open, 2, 0, 1, 2);


        retranslateUi(Form);

        QMetaObject::connectSlotsByName(Form);
    } // setupUi

    void retranslateUi(QWidget *Form)
    {
        Form->setWindowTitle(QCoreApplication::translate("Form", "Form", nullptr));
        label_rgb->setText(QCoreApplication::translate("Form", "TextLabel", nullptr));
        btn_start->setText(QCoreApplication::translate("Form", "\345\274\200\345\247\213\345\275\225\345\203\217", nullptr));
        label_depth->setText(QCoreApplication::translate("Form", "TextLabel", nullptr));
        btn_open->setText(QCoreApplication::translate("Form", "\345\274\200\345\220\257\346\221\204\345\203\217\345\244\264", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Form: public Ui_Form {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TRAIN_H
