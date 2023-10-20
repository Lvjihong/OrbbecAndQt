/********************************************************************************
** Form generated from reading UI file 'MainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_OrbbecDemoClass
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QPushButton *btn_depth;
    QPushButton *btn_save_depth_png;
    QPushButton *btn_save_color_png;
    QPushButton *btn_stop_color;
    QPushButton *btn_color;
    QLabel *label_color;
    QPushButton *btn_stop_depth;
    QLabel *label_depth;
    QPushButton *btn_save_depth_avi;
    QPushButton *btn_save_color_avi;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *OrbbecDemoClass)
    {
        if (OrbbecDemoClass->objectName().isEmpty())
            OrbbecDemoClass->setObjectName(QString::fromUtf8("OrbbecDemoClass"));
        OrbbecDemoClass->resize(1142, 606);
        centralWidget = new QWidget(OrbbecDemoClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        btn_depth = new QPushButton(centralWidget);
        btn_depth->setObjectName(QString::fromUtf8("btn_depth"));

        gridLayout->addWidget(btn_depth, 0, 0, 1, 1);

        btn_save_depth_png = new QPushButton(centralWidget);
        btn_save_depth_png->setObjectName(QString::fromUtf8("btn_save_depth_png"));

        gridLayout->addWidget(btn_save_depth_png, 1, 0, 1, 1);

        btn_save_color_png = new QPushButton(centralWidget);
        btn_save_color_png->setObjectName(QString::fromUtf8("btn_save_color_png"));

        gridLayout->addWidget(btn_save_color_png, 1, 1, 1, 1);

        btn_stop_color = new QPushButton(centralWidget);
        btn_stop_color->setObjectName(QString::fromUtf8("btn_stop_color"));

        gridLayout->addWidget(btn_stop_color, 3, 1, 1, 1);

        btn_color = new QPushButton(centralWidget);
        btn_color->setObjectName(QString::fromUtf8("btn_color"));

        gridLayout->addWidget(btn_color, 0, 1, 1, 1);

        label_color = new QLabel(centralWidget);
        label_color->setObjectName(QString::fromUtf8("label_color"));

        gridLayout->addWidget(label_color, 4, 1, 1, 1);

        btn_stop_depth = new QPushButton(centralWidget);
        btn_stop_depth->setObjectName(QString::fromUtf8("btn_stop_depth"));

        gridLayout->addWidget(btn_stop_depth, 3, 0, 1, 1);

        label_depth = new QLabel(centralWidget);
        label_depth->setObjectName(QString::fromUtf8("label_depth"));

        gridLayout->addWidget(label_depth, 4, 0, 1, 1);

        btn_save_depth_avi = new QPushButton(centralWidget);
        btn_save_depth_avi->setObjectName(QString::fromUtf8("btn_save_depth_avi"));

        gridLayout->addWidget(btn_save_depth_avi, 2, 0, 1, 1);

        btn_save_color_avi = new QPushButton(centralWidget);
        btn_save_color_avi->setObjectName(QString::fromUtf8("btn_save_color_avi"));

        gridLayout->addWidget(btn_save_color_avi, 2, 1, 1, 1);

        OrbbecDemoClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(OrbbecDemoClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1142, 26));
        OrbbecDemoClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(OrbbecDemoClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        OrbbecDemoClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(OrbbecDemoClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        OrbbecDemoClass->setStatusBar(statusBar);

        retranslateUi(OrbbecDemoClass);

        QMetaObject::connectSlotsByName(OrbbecDemoClass);
    } // setupUi

    void retranslateUi(QMainWindow *OrbbecDemoClass)
    {
        OrbbecDemoClass->setWindowTitle(QCoreApplication::translate("OrbbecDemoClass", "OrbbecDemo", nullptr));
        btn_depth->setText(QCoreApplication::translate("OrbbecDemoClass", "\346\267\261\345\272\246\345\233\276\345\203\217", nullptr));
        btn_save_depth_png->setText(QCoreApplication::translate("OrbbecDemoClass", "\344\277\235\345\255\23030\345\270\247\346\267\261\345\272\246\345\233\276\345\203\217", nullptr));
        btn_save_color_png->setText(QCoreApplication::translate("OrbbecDemoClass", "\344\277\235\345\255\23030\345\270\247RGB\345\233\276\345\203\217", nullptr));
        btn_stop_color->setText(QCoreApplication::translate("OrbbecDemoClass", "\347\273\223\346\235\237rgb\345\233\276\345\203\217\350\247\206\351\242\221\345\275\225\345\210\266", nullptr));
        btn_color->setText(QCoreApplication::translate("OrbbecDemoClass", "rgb\345\233\276\345\203\217", nullptr));
        label_color->setText(QCoreApplication::translate("OrbbecDemoClass", "TextLabel", nullptr));
        btn_stop_depth->setText(QCoreApplication::translate("OrbbecDemoClass", "\347\273\223\346\235\237\346\267\261\345\272\246\345\233\276\345\203\217\350\247\206\351\242\221\345\275\225\345\210\266", nullptr));
        label_depth->setText(QCoreApplication::translate("OrbbecDemoClass", "TextLabel", nullptr));
        btn_save_depth_avi->setText(QCoreApplication::translate("OrbbecDemoClass", "\345\274\200\345\247\213\346\267\261\345\272\246\345\233\276\345\203\217\350\247\206\351\242\221\345\275\225\345\210\266", nullptr));
        btn_save_color_avi->setText(QCoreApplication::translate("OrbbecDemoClass", "\345\274\200\345\247\213rgb\345\233\276\345\203\217\350\247\206\351\242\221\345\275\225\345\210\266", nullptr));
    } // retranslateUi

};

namespace Ui {
    class OrbbecDemoClass: public Ui_OrbbecDemoClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
