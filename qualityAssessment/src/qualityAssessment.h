#ifndef RenderWindowUISingleInheritance_H
#define RenderWindowUISingleInheritance_H
#include <QApplication>
#include <QSurfaceFormat>
#include <QVTKOpenGLWidget.h>
#include <vtkSmartPointer.h>
#include <QPushButton>
#include <QMainWindow>
 #include <QThread>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkDataSetReader.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>
#include <vtkCamera.h>
#include <QMainWindow>
#include <QPushButton>
#include <QCoreApplication>
#include <QtConcurrent/qtconcurrentrun.h>
#include <QThread>
#include <QString>
#include <QMutex>
#include <QDebug>
#include <fstream>
#include <QtWidgets/QRadioButton>
#include <QKeyEvent>
#include <QWidget>
#include <QtGui>
#include <QMouseEvent>


// Forward Qt class declarations
class Ui_qualityAssessment;


class qualityAssessment : public QMainWindow
{
  Q_OBJECT
public:
 
  // Constructor/Destructor
  qualityAssessment(); 

  ~qualityAssessment() {}
  Ui_qualityAssessment *ui;



public slots:
 void next();
 void Checked();
 void keyPressEvent(QKeyEvent*);
 void interruptHandler();

// void renderNext();
  virtual void slotExit();

private:
 
  // Designer form
};





#endif
