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
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QTreeView>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_TrainWindow
{
public:
    QHBoxLayout *horizontalLayout_2;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QHBoxLayout *horizontalLayout_3;
    QTreeView *treeView;
    QWidget *widget_2;
    QGridLayout *gridLayout;
    QPushButton *btn_open;
    QWidget *widget;
    QHBoxLayout *horizontalLayout;
    QLabel *label_depth;
    QLabel *label_rgb;
    QPushButton *btn_start;

    void setupUi(QWidget *TrainWindow)
    {
        if (TrainWindow->objectName().isEmpty())
            TrainWindow->setObjectName(QString::fromUtf8("TrainWindow"));
        TrainWindow->resize(1072, 547);
        TrainWindow->setMinimumSize(QSize(1072, 547));
        horizontalLayout_2 = new QHBoxLayout(TrainWindow);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        scrollArea = new QScrollArea(TrainWindow);
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::MinimumExpanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(scrollArea->sizePolicy().hasHeightForWidth());
        scrollArea->setSizePolicy(sizePolicy);
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 278, 523));
        horizontalLayout_3 = new QHBoxLayout(scrollAreaWidgetContents);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        treeView = new QTreeView(scrollAreaWidgetContents);
        treeView->setObjectName(QString::fromUtf8("treeView"));
        sizePolicy.setHeightForWidth(treeView->sizePolicy().hasHeightForWidth());
        treeView->setSizePolicy(sizePolicy);

        horizontalLayout_3->addWidget(treeView);

        scrollArea->setWidget(scrollAreaWidgetContents);

        horizontalLayout_2->addWidget(scrollArea);

        widget_2 = new QWidget(TrainWindow);
        widget_2->setObjectName(QString::fromUtf8("widget_2"));
        QSizePolicy sizePolicy1(QSizePolicy::MinimumExpanding, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(widget_2->sizePolicy().hasHeightForWidth());
        widget_2->setSizePolicy(sizePolicy1);
        widget_2->setMaximumSize(QSize(16777215, 16777215));
        gridLayout = new QGridLayout(widget_2);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        btn_open = new QPushButton(widget_2);
        btn_open->setObjectName(QString::fromUtf8("btn_open"));
        QSizePolicy sizePolicy2(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(btn_open->sizePolicy().hasHeightForWidth());
        btn_open->setSizePolicy(sizePolicy2);
        QFont font;
        font.setPointSize(18);
        font.setBold(true);
        font.setWeight(75);
        btn_open->setFont(font);

        gridLayout->addWidget(btn_open, 0, 0, 1, 1);

        widget = new QWidget(widget_2);
        widget->setObjectName(QString::fromUtf8("widget"));
        sizePolicy2.setHeightForWidth(widget->sizePolicy().hasHeightForWidth());
        widget->setSizePolicy(sizePolicy2);
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


        gridLayout->addWidget(widget, 1, 0, 1, 2);

        btn_start = new QPushButton(widget_2);
        btn_start->setObjectName(QString::fromUtf8("btn_start"));
        sizePolicy2.setHeightForWidth(btn_start->sizePolicy().hasHeightForWidth());
        btn_start->setSizePolicy(sizePolicy2);
        btn_start->setFont(font);

        gridLayout->addWidget(btn_start, 0, 1, 1, 1);

        gridLayout->setRowStretch(0, 1);
        gridLayout->setRowStretch(1, 4);

        horizontalLayout_2->addWidget(widget_2);

        horizontalLayout_2->setStretch(0, 1);
        horizontalLayout_2->setStretch(1, 5);

        retranslateUi(TrainWindow);

        QMetaObject::connectSlotsByName(TrainWindow);
    } // setupUi

    void retranslateUi(QWidget *TrainWindow)
    {
        TrainWindow->setWindowTitle(QCoreApplication::translate("TrainWindow", "\345\244\247\345\214\227\345\206\234\344\270\223\351\241\271_\351\207\207\351\233\206\350\256\255\347\273\203\346\225\260\346\215\256", nullptr));
        btn_open->setText(QCoreApplication::translate("TrainWindow", "\345\274\200\345\220\257\346\221\204\345\203\217\345\244\264", nullptr));
        label_depth->setText(QString());
        label_rgb->setText(QString());
        btn_start->setText(QCoreApplication::translate("TrainWindow", "\345\274\200\345\247\213\351\207\207\351\233\206\346\225\260\346\215\256", nullptr));
    } // retranslateUi

};

namespace Ui {
    class TrainWindow: public Ui_TrainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TRAIN_H
