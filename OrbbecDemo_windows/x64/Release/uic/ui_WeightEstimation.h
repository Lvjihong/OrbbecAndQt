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
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_WeightEstimation
{
public:
    QHBoxLayout *horizontalLayout_2;
    QLabel *label;
    QWidget *widget_2;
    QVBoxLayout *verticalLayout;
    QWidget *widget_3;
    QHBoxLayout *horizontalLayout_4;
    QPushButton *btn_start;
    QPushButton *btn_select;
    QPushButton *btn_exit;
    QWidget *widget_4;
    QHBoxLayout *horizontalLayout;
    QLabel *label_depth;

    void setupUi(QWidget *WeightEstimation)
    {
        if (WeightEstimation->objectName().isEmpty())
            WeightEstimation->setObjectName(QString::fromUtf8("WeightEstimation"));
        WeightEstimation->setWindowModality(Qt::WindowModal);
        WeightEstimation->resize(800, 522);
        horizontalLayout_2 = new QHBoxLayout(WeightEstimation);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
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

        horizontalLayout_2->addWidget(label);

        widget_2 = new QWidget(WeightEstimation);
        widget_2->setObjectName(QString::fromUtf8("widget_2"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(5);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(widget_2->sizePolicy().hasHeightForWidth());
        widget_2->setSizePolicy(sizePolicy);
        widget_2->setMinimumSize(QSize(100, 0));
        widget_2->setMaximumSize(QSize(16777215, 16777215));
        verticalLayout = new QVBoxLayout(widget_2);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        widget_3 = new QWidget(widget_2);
        widget_3->setObjectName(QString::fromUtf8("widget_3"));
        horizontalLayout_4 = new QHBoxLayout(widget_3);
        horizontalLayout_4->setSpacing(5);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(2, 2, 2, 2);
        btn_start = new QPushButton(widget_3);
        btn_start->setObjectName(QString::fromUtf8("btn_start"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(btn_start->sizePolicy().hasHeightForWidth());
        btn_start->setSizePolicy(sizePolicy1);
        QFont font1;
        font1.setFamily(QString::fromUtf8("\351\273\221\344\275\223"));
        font1.setPointSize(35);
        font1.setBold(true);
        font1.setWeight(75);
        btn_start->setFont(font1);

        horizontalLayout_4->addWidget(btn_start);

        btn_select = new QPushButton(widget_3);
        btn_select->setObjectName(QString::fromUtf8("btn_select"));
        sizePolicy1.setHeightForWidth(btn_select->sizePolicy().hasHeightForWidth());
        btn_select->setSizePolicy(sizePolicy1);
        btn_select->setFont(font1);

        horizontalLayout_4->addWidget(btn_select);

        btn_exit = new QPushButton(widget_3);
        btn_exit->setObjectName(QString::fromUtf8("btn_exit"));
        sizePolicy1.setHeightForWidth(btn_exit->sizePolicy().hasHeightForWidth());
        btn_exit->setSizePolicy(sizePolicy1);
        btn_exit->setFont(font1);

        horizontalLayout_4->addWidget(btn_exit);

        horizontalLayout_4->setStretch(0, 3);
        horizontalLayout_4->setStretch(1, 3);
        horizontalLayout_4->setStretch(2, 2);

        verticalLayout->addWidget(widget_3);

        widget_4 = new QWidget(widget_2);
        widget_4->setObjectName(QString::fromUtf8("widget_4"));
        QSizePolicy sizePolicy2(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(widget_4->sizePolicy().hasHeightForWidth());
        widget_4->setSizePolicy(sizePolicy2);
        horizontalLayout = new QHBoxLayout(widget_4);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label_depth = new QLabel(widget_4);
        label_depth->setObjectName(QString::fromUtf8("label_depth"));
        label_depth->setMinimumSize(QSize(72, 15));
        label_depth->setAlignment(Qt::AlignCenter);

        horizontalLayout->addWidget(label_depth);


        verticalLayout->addWidget(widget_4);

        verticalLayout->setStretch(0, 1);
        verticalLayout->setStretch(1, 8);

        horizontalLayout_2->addWidget(widget_2);


        retranslateUi(WeightEstimation);

        QMetaObject::connectSlotsByName(WeightEstimation);
    } // setupUi

    void retranslateUi(QWidget *WeightEstimation)
    {
        WeightEstimation->setWindowTitle(QCoreApplication::translate("WeightEstimation", "\345\244\247\345\214\227\345\206\234\344\270\223\351\241\271_\347\214\252\344\275\223\351\207\215\344\274\260\350\256\241", nullptr));
        label->setText(QCoreApplication::translate("WeightEstimation", "0\345\205\254\346\226\244", nullptr));
        btn_start->setText(QCoreApplication::translate("WeightEstimation", "\345\274\200\345\247\213\344\274\260\351\207\215", nullptr));
        btn_select->setText(QCoreApplication::translate("WeightEstimation", "\351\200\211\346\213\251\346\226\207\344\273\266", nullptr));
        btn_exit->setText(QCoreApplication::translate("WeightEstimation", "\351\200\200\345\207\272", nullptr));
        label_depth->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class WeightEstimation: public Ui_WeightEstimation {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WEIGHTESTIMATION_H
