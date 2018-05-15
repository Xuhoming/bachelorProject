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
#include <vtkCamera.h>
#include <QMainWindow>
#include <QPushButton>
#include <QCoreApplication>
#include <vtkVertexGlyphFilter.h>

vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
vtkNew<vtkRenderer> rendererLeft;
vtkNew<vtkRenderer> rendererRight;
vtkSmartPointer<vtkActor> actorLeft =vtkSmartPointer<vtkActor>::New();
vtkSmartPointer<vtkActor> actorRight =vtkSmartPointer<vtkActor>::New();


void qualityAssessment::leftButton()
 {
    cout<<"left"<<endl;
    vtkSmartPointer<vtkPLYReader> readerLeft = vtkSmartPointer<vtkPLYReader>::New();
    vtkSmartPointer<vtkPLYReader> readerRight = vtkSmartPointer<vtkPLYReader>::New();
    readerLeft->SetFileName ( "best_andrew_disk_NS2_NN1.ply");
    // Visualize

    vtkSmartPointer<vtkPolyDataMapper> mapperLeft = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapperLeft->SetInputConnection(readerLeft->GetOutputPort());
    actorLeft->SetMapper(mapperLeft);

    vtkSmartPointer<vtkPolyDataMapper> mapperRight = vtkSmartPointer<vtkPolyDataMapper>::New();
    readerRight->SetFileName ("best_andrew_disk_NS2_NN1.ply" );
    mapperRight->SetInputConnection(readerRight->GetOutputPort());
    actorRight->SetMapper(mapperRight);

    actorLeft->GetProperty()->LightingOff();
    actorRight->GetProperty()->LightingOff();

    renderWindow->Render();
    rendererLeft->ResetCamera();
 }

void qualityAssessment::rightButton()
 {
    cout<<"right"<<endl;
 }

// Constructor
qualityAssessment::qualityAssessment()
{

  this->ui = new Ui_qualityAssessment;
  this->ui->setupUi(this);

  std::string inputFilenameLeft = "best_dragon_disk_NS5_NN7.ply";
  std::string inputFilenameRight = "dragon.ply";

  vtkSmartPointer<vtkPLYReader> readerLeft = vtkSmartPointer<vtkPLYReader>::New();
  readerLeft->SetFileName ( inputFilenameLeft.c_str() );
  vtkSmartPointer<vtkPLYReader> readerRight = vtkSmartPointer<vtkPLYReader>::New();
  readerRight->SetFileName ( inputFilenameRight.c_str() );



  this->ui->qvtkWidget->SetRenderWindow(renderWindow);

  connect (this->ui->leftBest,  SIGNAL (clicked ()), this, SLOT (leftButton()));
  connect (this->ui->rightBest,  SIGNAL (clicked ()), this, SLOT (rightButton()));


  vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
  vertexGlyphFilter->SetInputConnection(readerRight->GetOutputPort());
  vertexGlyphFilter->Update();

  // Visualize
  vtkSmartPointer<vtkPolyDataMapper> mapperLeft =vtkSmartPointer<vtkPolyDataMapper>::New();
  mapperLeft->SetInputConnection(readerLeft->GetOutputPort());
  vtkSmartPointer<vtkPolyDataMapper> mapperRight =vtkSmartPointer<vtkPolyDataMapper>::New();
  mapperRight->SetInputConnection(vertexGlyphFilter->GetOutputPort());

  cout<<readerLeft->GetOutput()->GetNumberOfCells()<<endl;


  actorLeft->SetMapper(mapperLeft);

  actorRight->SetMapper(mapperRight);
  vtkSmartPointer<vtkCamera> sharedCamera = vtkSmartPointer<vtkCamera>::New();
  //actor->GetProperty()->LightingOff();
  // VTK Renderer

  rendererLeft->SetViewport(0,0,0.5,1);
  rendererLeft->SetActiveCamera(sharedCamera);
  rendererLeft->AddActor(actorLeft);

  rendererRight->SetViewport(0.5,0,1,1);
  rendererRight->SetActiveCamera(sharedCamera);
  rendererLeft->ResetCamera();
  rendererRight->SetBackground(.3,.3,.3);
  rendererRight->AddActor(actorRight);

  // VTK/Qt wedded
  this->ui->qvtkWidget->GetRenderWindow()->AddRenderer(rendererLeft);
   this->ui->qvtkWidget->GetRenderWindow()->AddRenderer(rendererRight);
this->ui->qvtkWidget->GetRenderWindow()->FullScreenOn();
  // Set up action signals and slots
  connect(this->ui->actionExit, SIGNAL(triggered()), this, SLOT(slotExit()));
}

void qualityAssessment::slotExit()
{
  qApp->exit();
}
