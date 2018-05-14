#include "qualityAssessment.h"

// This is included here because it is forward declared in
// RenderWindowUISingleInheritance.h
#include "ui_qualityAssessment.h"
#include "../build/ui_qualityAssessment.h"
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkPLYReader.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>
#include <QMainWindow>
#include <QPushButton>
#include <QCoreApplication>
void qualityAssessment::leftButton()
 {
    cout<<"left"<<endl;
 }
void qualityAssessment::rightButton()
 {
    cout<<"right"<<endl;
 }

// Constructor
qualityAssessment::qualityAssessment()
{
  std::string inputFilename = "best_dragon_disk_NS5_NN7.ply";
  vtkSmartPointer<vtkPLYReader> reader =
  vtkSmartPointer<vtkPLYReader>::New();
  reader->SetFileName ( inputFilename.c_str() );
  this->ui = new Ui_qualityAssessment;
  this->ui->setupUi(this);

  vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
  this->ui->qvtkWidget->SetRenderWindow(renderWindow);

  connect (this->ui->leftBest,  SIGNAL (clicked ()), this, SLOT (leftButton()));
  connect (this->ui->rightBest,  SIGNAL (clicked ()), this, SLOT (rightButton()));

  // Visualize
  vtkSmartPointer<vtkPolyDataMapper> mapper =
  vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(reader->GetOutputPort());

  vtkSmartPointer<vtkActor> actor =
  vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  //actor->GetProperty()->LightingOff();
  // VTK Renderer
  vtkNew<vtkRenderer> renderer;
  renderer->AddActor(actor);

  // VTK/Qt wedded
  this->ui->qvtkWidget->GetRenderWindow()->AddRenderer(renderer);
this->ui->qvtkWidget->GetRenderWindow()->FullScreenOn();
  // Set up action signals and slots
  connect(this->ui->actionExit, SIGNAL(triggered()), this, SLOT(slotExit()));

}

void qualityAssessment::slotExit()
{
  qApp->exit();
}
