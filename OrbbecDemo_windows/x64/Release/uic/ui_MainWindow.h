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
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_OrbbecDemoClass
{
public:
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout;
    QPushButton *btn_train;
    QPushButton *btn_estimation;

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
        btn_train = new QPushButton(centralWidget);
        btn_train->setObjectName(QString::fromUtf8("btn_train"));
        btn_train->setEnabled(true);
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(btn_train->sizePolicy().hasHeightForWidth());
        btn_train->setSizePolicy(sizePolicy);
        QFont font;
        font.setFamily(QString::fromUtf8("\351\273\221\344\275\223"));
        font.setPointSize(35);
        font.setBold(true);
        font.setWeight(75);
        btn_train->setFont(font);
        btn_train->setIconSize(QSize(50, 50));
        btn_train->setCheckable(false);

        horizontalLayout->addWidget(btn_train);

        btn_estimation = new QPushButton(centralWidget);
        btn_estimation->setObjectName(QString::fromUtf8("btn_estimation"));
        sizePolicy.setHeightForWidth(btn_estimation->sizePolicy().hasHeightForWidth());
        btn_estimation->setSizePolicy(sizePolicy);
        btn_estimation->setFont(font);

        horizontalLayout->addWidget(btn_estimation);

        OrbbecDemoClass->setCentralWidget(centralWidget);

        retranslateUi(OrbbecDemoClass);

        QMetaObject::connectSlotsByName(OrbbecDemoClass);
    } // setupUi

    void retranslateUi(QMainWindow *OrbbecDemoClass)
    {
        OrbbecDemoClass->setWindowTitle(QCoreApplication::translate("OrbbecDemoClass", "OrbbecDemo", nullptr));
        btn_train->setText(QCoreApplication::translate("OrbbecDemoClass", "\350\256\255\347\273\203\346\250\241\345\274\217", nullptr));
        btn_estimation->setText(QCoreApplication::translate("OrbbecDemoClass", "\344\274\260\351\207\215\346\250\241\345\274\217", nullptr));
    } // retranslateUi

};

namespace Ui {
    class OrbbecDemoClass: public Ui_OrbbecDemoClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
