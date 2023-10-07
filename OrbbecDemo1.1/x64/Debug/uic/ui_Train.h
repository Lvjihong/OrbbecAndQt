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
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_TrainWindow
{
public:
    QGridLayout *gridLayout;
    QWidget *widget;
    QHBoxLayout *horizontalLayout;
    QLabel *label_depth;
    QLabel *label_rgb;
    QSpacerItem *horizontalSpacer_4;
    QSpacerItem *verticalSpacer;
    QPushButton *btn_start;
    QPushButton *btn_open;
    QSpacerItem *horizontalSpacer_2;
    QSpacerItem *horizontalSpacer;
    QSpacerItem *horizontalSpacer_3;
    QSpacerItem *verticalSpacer_2;

    void setupUi(QWidget *TrainWindow)
    {
        if (TrainWindow->objectName().isEmpty())
            TrainWindow->setObjectName(QString::fromUtf8("TrainWindow"));
        TrainWindow->resize(1072, 547);
        TrainWindow->setMinimumSize(QSize(1072, 547));
        gridLayout = new QGridLayout(TrainWindow);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        widget = new QWidget(TrainWindow);
        widget->setObjectName(QString::fromUtf8("widget"));
        horizontalLayout = new QHBoxLayout(widget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label_depth = new QLabel(widget);
        label_depth->setObjectName(QString::fromUtf8("label_depth"));
        label_depth->setMinimumSize(QSize(72, 15));
        label_depth->setAlignment(Qt::AlignCenter);

        horizontalLayout->addWidget(label_depth);

        label_rgb = new QLabel(widget);
        label_rgb->setObjectName(QString::fromUtf8("label_rgb"));
        label_rgb->setMinimumSize(QSize(72, 15));
        label_rgb->setAlignment(Qt::AlignCenter);

        horizontalLayout->addWidget(label_rgb);


        gridLayout->addWidget(widget, 4, 0, 1, 3);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer_4, 3, 2, 1, 1);

        verticalSpacer = new QSpacerItem(5, 5, QSizePolicy::Minimum, QSizePolicy::Fixed);

        gridLayout->addItem(verticalSpacer, 2, 1, 1, 1);

        btn_start = new QPushButton(TrainWindow);
        btn_start->setObjectName(QString::fromUtf8("btn_start"));

        gridLayout->addWidget(btn_start, 3, 1, 1, 1);

        btn_open = new QPushButton(TrainWindow);
        btn_open->setObjectName(QString::fromUtf8("btn_open"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(btn_open->sizePolicy().hasHeightForWidth());
        btn_open->setSizePolicy(sizePolicy);

        gridLayout->addWidget(btn_open, 1, 1, 1, 1);

        horizontalSpacer_2 = new QSpacerItem(20, 20, QSizePolicy::Preferred, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer_2, 1, 2, 1, 1);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer, 1, 0, 1, 1);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer_3, 3, 0, 1, 1);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Fixed);

        gridLayout->addItem(verticalSpacer_2, 0, 1, 1, 1);


        retranslateUi(TrainWindow);

        QMetaObject::connectSlotsByName(TrainWindow);
    } // setupUi

    void retranslateUi(QWidget *TrainWindow)
    {
        TrainWindow->setWindowTitle(QCoreApplication::translate("TrainWindow", "\345\244\247\345\214\227\345\206\234\344\270\223\351\241\271_\350\256\255\347\273\203\346\225\260\346\215\256\346\224\266\351\233\206", nullptr));
        label_depth->setText(QString());
        label_rgb->setText(QString());
        btn_start->setText(QCoreApplication::translate("TrainWindow", "\345\274\200\345\247\213\345\275\225\345\203\217", nullptr));
        btn_open->setText(QCoreApplication::translate("TrainWindow", "\345\274\200\345\220\257\346\221\204\345\203\217\345\244\264", nullptr));
    } // retranslateUi

};

namespace Ui {
    class TrainWindow: public Ui_TrainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TRAIN_H
