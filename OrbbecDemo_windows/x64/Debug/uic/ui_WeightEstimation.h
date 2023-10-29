/********************************************************************************
** Form generated from reading UI file 'WeightEstimation.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WEIGHTESTIMATION_H
#define UI_WEIGHTESTIMATION_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_WeightEstimation
{
public:
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QWidget *widget;

    void setupUi(QWidget *WeightEstimation)
    {
        if (WeightEstimation->objectName().isEmpty())
            WeightEstimation->setObjectName(QString::fromUtf8("WeightEstimation"));
        WeightEstimation->setWindowModality(Qt::WindowModal);
        WeightEstimation->resize(800, 522);
        horizontalLayout = new QHBoxLayout(WeightEstimation);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(WeightEstimation);
        label->setObjectName(QString::fromUtf8("label"));
        QFont font;
        font.setFamily(QString::fromUtf8("\351\273\221\344\275\223"));
        font.setPointSize(25);
        font.setBold(true);
        font.setWeight(75);
        label->setFont(font);
        label->setAlignment(Qt::AlignCenter);
        label->setWordWrap(false);

        horizontalLayout->addWidget(label);

        widget = new QWidget(WeightEstimation);
        widget->setObjectName(QString::fromUtf8("widget"));

        horizontalLayout->addWidget(widget);

        horizontalLayout->setStretch(0, 1);
        horizontalLayout->setStretch(1, 10);

        retranslateUi(WeightEstimation);

        QMetaObject::connectSlotsByName(WeightEstimation);
    } // setupUi

    void retranslateUi(QWidget *WeightEstimation)
    {
        WeightEstimation->setWindowTitle(QCoreApplication::translate("WeightEstimation", "\345\244\247\345\214\227\345\206\234\344\270\223\351\241\271_\347\214\252\344\275\223\351\207\215\344\274\260\350\256\241", nullptr));
        label->setText(QCoreApplication::translate("WeightEstimation", "95\345\205\254\346\226\244", nullptr));
    } // retranslateUi

};

namespace Ui {
    class WeightEstimation: public Ui_WeightEstimation {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WEIGHTESTIMATION_H
