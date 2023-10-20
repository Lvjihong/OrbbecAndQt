/********************************************************************************
** Form generated from reading UI file 'TrainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TRAINWINDOW_H
#define UI_TRAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    QLabel *label_depth;
    QLabel *label_rgb;
    QPushButton *btn_start;
    QPushButton *btn_stop;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(997, 600);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label_depth = new QLabel(centralwidget);
        label_depth->setObjectName(QString::fromUtf8("label_depth"));

        gridLayout->addWidget(label_depth, 3, 0, 1, 1);

        label_rgb = new QLabel(centralwidget);
        label_rgb->setObjectName(QString::fromUtf8("label_rgb"));

        gridLayout->addWidget(label_rgb, 3, 1, 1, 1);

        btn_start = new QPushButton(centralwidget);
        btn_start->setObjectName(QString::fromUtf8("btn_start"));

        gridLayout->addWidget(btn_start, 1, 0, 1, 2);

        btn_stop = new QPushButton(centralwidget);
        btn_stop->setObjectName(QString::fromUtf8("btn_stop"));

        gridLayout->addWidget(btn_stop, 2, 0, 1, 2);

        MainWindow->setCentralWidget(centralwidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        label_depth->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label_rgb->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        btn_start->setText(QCoreApplication::translate("MainWindow", "PushButton", nullptr));
        btn_stop->setText(QCoreApplication::translate("MainWindow", "PushButton", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TRAINWINDOW_H
