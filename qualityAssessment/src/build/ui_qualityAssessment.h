/********************************************************************************
** Form generated from reading UI file 'qualityAssessment.ui'
**
** Created by: Qt User Interface Compiler version 5.10.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_QUALITYASSESSMENT_H
#define UI_QUALITYASSESSMENT_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "QVTKOpenGLWidget.h"

QT_BEGIN_NAMESPACE

class Ui_qualityAssessment
{
public:
    QAction *actionOpenFile;
    QAction *actionExit;
    QAction *actionPrint;
    QAction *actionHelp;
    QAction *actionSave;
    QWidget *centralwidget;
    QVTKOpenGLWidget *qvtkWidget;
    QLabel *thanks;
    QPushButton *nextbutton;
    QLabel *label_4;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *verticalLayout;
    QRadioButton *leftratio;
    QRadioButton *rightratio;
    QRadioButton *equalratio;

    void setupUi(QMainWindow *qualityAssessment)
    {
        if (qualityAssessment->objectName().isEmpty())
            qualityAssessment->setObjectName(QStringLiteral("qualityAssessment"));
        qualityAssessment->setEnabled(true);
        qualityAssessment->resize(2560, 1600);
        actionOpenFile = new QAction(qualityAssessment);
        actionOpenFile->setObjectName(QStringLiteral("actionOpenFile"));
        actionOpenFile->setEnabled(true);
        actionExit = new QAction(qualityAssessment);
        actionExit->setObjectName(QStringLiteral("actionExit"));
        actionPrint = new QAction(qualityAssessment);
        actionPrint->setObjectName(QStringLiteral("actionPrint"));
        actionHelp = new QAction(qualityAssessment);
        actionHelp->setObjectName(QStringLiteral("actionHelp"));
        actionSave = new QAction(qualityAssessment);
        actionSave->setObjectName(QStringLiteral("actionSave"));
        centralwidget = new QWidget(qualityAssessment);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        qvtkWidget = new QVTKOpenGLWidget(centralwidget);
        qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));
        qvtkWidget->setGeometry(QRect(0, -1, 2410, 1600));
        thanks = new QLabel(centralwidget);
        thanks->setObjectName(QStringLiteral("thanks"));
        thanks->setGeometry(QRect(600, 180, 1320, 1080));
        QFont font;
        font.setPointSize(61);
        font.setBold(false);
        font.setWeight(50);
        font.setKerning(true);
        font.setStyleStrategy(QFont::PreferDefault);
        thanks->setFont(font);
        thanks->setAlignment(Qt::AlignCenter);
        nextbutton = new QPushButton(centralwidget);
        nextbutton->setObjectName(QStringLiteral("nextbutton"));
        nextbutton->setGeometry(QRect(2430, 1200, 131, 41));
        nextbutton->setAutoFillBackground(false);
        nextbutton->setFlat(false);
        label_4 = new QLabel(centralwidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setEnabled(true);
        label_4->setGeometry(QRect(2420, 560, 133, 51));
        QFont font1;
        font1.setPointSize(17);
        label_4->setFont(font1);
        label_4->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        verticalLayoutWidget_2 = new QWidget(centralwidget);
        verticalLayoutWidget_2->setObjectName(QStringLiteral("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(2420, 620, 137, 261));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget_2);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        leftratio = new QRadioButton(verticalLayoutWidget_2);
        leftratio->setObjectName(QStringLiteral("leftratio"));
        QFont font2;
        font2.setPointSize(18);
        font2.setBold(false);
        font2.setWeight(50);
        leftratio->setFont(font2);
        leftratio->setLayoutDirection(Qt::LeftToRight);
        leftratio->setAutoFillBackground(false);

        verticalLayout->addWidget(leftratio);

        rightratio = new QRadioButton(verticalLayoutWidget_2);
        rightratio->setObjectName(QStringLiteral("rightratio"));
        QFont font3;
        font3.setPointSize(18);
        rightratio->setFont(font3);

        verticalLayout->addWidget(rightratio);

        equalratio = new QRadioButton(verticalLayoutWidget_2);
        equalratio->setObjectName(QStringLiteral("equalratio"));
        equalratio->setFont(font2);

        verticalLayout->addWidget(equalratio);

        qualityAssessment->setCentralWidget(centralwidget);
        qvtkWidget->raise();
        thanks->raise();
        nextbutton->raise();
        verticalLayoutWidget_2->raise();
        label_4->raise();

        retranslateUi(qualityAssessment);

        QMetaObject::connectSlotsByName(qualityAssessment);
    } // setupUi

    void retranslateUi(QMainWindow *qualityAssessment)
    {
        qualityAssessment->setWindowTitle(QApplication::translate("qualityAssessment", "qualityAssessment", nullptr));
        actionOpenFile->setText(QApplication::translate("qualityAssessment", "Open File...", nullptr));
        actionExit->setText(QApplication::translate("qualityAssessment", "Exit", nullptr));
        actionPrint->setText(QApplication::translate("qualityAssessment", "Print", nullptr));
        actionHelp->setText(QApplication::translate("qualityAssessment", "Help", nullptr));
        actionSave->setText(QApplication::translate("qualityAssessment", "Save", nullptr));
        thanks->setText(QApplication::translate("qualityAssessment", "Thank you for your participation!", nullptr));
        nextbutton->setText(QApplication::translate("qualityAssessment", "Next", nullptr));
        label_4->setText(QApplication::translate("qualityAssessment", "Please select the \n"
"model you prefer", nullptr));
        leftratio->setText(QApplication::translate("qualityAssessment", "Left", nullptr));
        rightratio->setText(QApplication::translate("qualityAssessment", "Right", nullptr));
        equalratio->setText(QApplication::translate("qualityAssessment", "No difference", nullptr));
    } // retranslateUi

};

namespace Ui {
    class qualityAssessment: public Ui_qualityAssessment {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QUALITYASSESSMENT_H
