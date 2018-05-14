#-------------------------------------------------
#
# Project created by QtCreator 2018-05-11T16:47:09
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = testQVTK
TEMPLATE = app


SOURCES +="qualityAssessment.cxx"
SOURCES +="qualityAssessmentDriver.cxx"
INCLUDEPATH +="/usr/local/include/vtk-8.1"
HEADERS  +="qualityAssessment.h"

FORMS    +="qualityAssessment.ui"
