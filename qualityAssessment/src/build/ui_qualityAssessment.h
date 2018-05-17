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
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
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
    QRadioButton *leftratio;
    QRadioButton *rightratio;
    QRadioButton *equalratio;
    QPushButton *nextbutton;

    void setupUi(QMainWindow *qualityAssessment)
    {
        if (qualityAssessment->objectName().isEmpty())
            qualityAssessment->setObjectName(QStringLiteral("qualityAssessment"));
        qualityAssessment->setEnabled(true);
        qualityAssessment->resize(1600, 900);
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
        qvtkWidget->setGeometry(QRect(0, 0, 1351, 900));
        leftratio = new QRadioButton(centralwidget);
        leftratio->setObjectName(QStringLiteral("leftratio"));
        leftratio->setGeometry(QRect(1460, 370, 117, 22));
        rightratio = new QRadioButton(centralwidget);
        rightratio->setObjectName(QStringLiteral("rightratio"));
        rightratio->setGeometry(QRect(1460, 440, 117, 22));
        equalratio = new QRadioButton(centralwidget);
        equalratio->setObjectName(QStringLiteral("equalratio"));
        equalratio->setGeometry(QRect(1460, 520, 117, 22));
        nextbutton = new QPushButton(centralwidget);
        nextbutton->setObjectName(QStringLiteral("nextbutton"));
        nextbutton->setGeometry(QRect(1470, 820, 99, 27));
        qualityAssessment->setCentralWidget(centralwidget);

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
        leftratio->setText(QApplication::translate("qualityAssessment", "left ", nullptr));
        rightratio->setText(QApplication::translate("qualityAssessment", "right", nullptr));
        equalratio->setText(QApplication::translate("qualityAssessment", "equal", nullptr));
        nextbutton->setText(QApplication::translate("qualityAssessment", "next", nullptr));
    } // retranslateUi

};

namespace Ui {
    class qualityAssessment: public Ui_qualityAssessment {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QUALITYASSESSMENT_H
