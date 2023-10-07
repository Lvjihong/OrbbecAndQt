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
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_OrbbecDemoClass
{
public:
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer;
    QPushButton *btn_train;
    QSpacerItem *horizontalSpacer_3;
    QPushButton *pushButton_2;
    QSpacerItem *horizontalSpacer_2;

    void setupUi(QMainWindow *OrbbecDemoClass)
    {
        if (OrbbecDemoClass->objectName().isEmpty())
            OrbbecDemoClass->setObjectName(QString::fromUtf8("OrbbecDemoClass"));
        OrbbecDemoClass->resize(600, 400);
        centralWidget = new QWidget(OrbbecDemoClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        horizontalLayout = new QHBoxLayout(centralWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        btn_train = new QPushButton(centralWidget);
        btn_train->setObjectName(QString::fromUtf8("btn_train"));

        horizontalLayout->addWidget(btn_train);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_3);

        pushButton_2 = new QPushButton(centralWidget);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));

        horizontalLayout->addWidget(pushButton_2);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_2);

        OrbbecDemoClass->setCentralWidget(centralWidget);

        retranslateUi(OrbbecDemoClass);

        QMetaObject::connectSlotsByName(OrbbecDemoClass);
    } // setupUi

    void retranslateUi(QMainWindow *OrbbecDemoClass)
    {
        OrbbecDemoClass->setWindowTitle(QCoreApplication::translate("OrbbecDemoClass", "OrbbecDemo", nullptr));
        btn_train->setText(QCoreApplication::translate("OrbbecDemoClass", "\350\256\255\347\273\203\346\250\241\345\274\217", nullptr));
        pushButton_2->setText(QCoreApplication::translate("OrbbecDemoClass", "\344\274\260\351\207\215\346\250\241\345\274\217", nullptr));
    } // retranslateUi

};

namespace Ui {
    class OrbbecDemoClass: public Ui_OrbbecDemoClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
