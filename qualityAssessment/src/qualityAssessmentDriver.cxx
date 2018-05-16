#include "qualityAssessment.h"


// This is included here because it is forward declared in
// RenderWindowUISingleInheritance.h
#include "ui_qualityAssessment.h"
#include "../build/ui_qualityAssessment.h"

vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow=vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow2=vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
vtkSmartPointer<vtkRenderer> rendererLeft=vtkSmartPointer<vtkRenderer>::New();
vtkSmartPointer<vtkRenderer> rendererRight=vtkSmartPointer<vtkRenderer>::New();

vtkSmartPointer<vtkRenderer> rendererLeft2=vtkSmartPointer<vtkRenderer>::New();
vtkSmartPointer<vtkRenderer> rendererRight2=vtkSmartPointer<vtkRenderer>::New();


vtkSmartPointer<vtkActor> actorLeft =vtkSmartPointer<vtkActor>::New();
vtkSmartPointer<vtkActor> actorRight =vtkSmartPointer<vtkActor>::New();

vtkSmartPointer<vtkActor> actorLeft2 =vtkSmartPointer<vtkActor>::New();
vtkSmartPointer<vtkActor> actorRight2 =vtkSmartPointer<vtkActor>::New();

void renderNext(QVTKOpenGLWidget* qvtkWidget,vtkGenericOpenGLRenderWindow* renderWindow,qualityAssessment* qualityAssess){
    qDebug()<<"start multi-thread"<<endl;
    std::string inputFilenameLeft2 = "best_andrew_disk_NS2_NN1.ply";
    std::string inputFilenameRight2 = "270000.ply";
    vtkSmartPointer<vtkPLYReader> readerLeft2 = vtkSmartPointer<vtkPLYReader>::New();
    readerLeft2->SetFileName ( inputFilenameLeft2.c_str() );
    vtkSmartPointer<vtkPLYReader> readerRight2 = vtkSmartPointer<vtkPLYReader>::New();
    readerRight2->SetFileName ( inputFilenameRight2.c_str() );

    vtkSmartPointer<vtkPolyDataMapper> mapperLeft2 =vtkSmartPointer<vtkPolyDataMapper>::New();
    mapperLeft2->SetInputConnection(readerLeft2->GetOutputPort());
    vtkSmartPointer<vtkPolyDataMapper> mapperRight2 =vtkSmartPointer<vtkPolyDataMapper>::New();
    mapperRight2->SetInputConnection(readerRight2->GetOutputPort());

    actorLeft2->GetProperty()->LightingOff();
    actorRight2->GetProperty()->LightingOff();
    actorLeft2->SetMapper(mapperLeft2);
    actorRight2->SetMapper(mapperRight2);
    vtkSmartPointer<vtkCamera> sharedCamera2 = vtkSmartPointer<vtkCamera>::New();
    rendererLeft2->SetViewport(0,0,0.5,1);
    rendererLeft2->SetActiveCamera(sharedCamera2);
    rendererLeft2->AddActor(actorLeft2);

    rendererRight2->SetViewport(0.5,0,1,1);
    rendererRight2->SetActiveCamera(sharedCamera2);
    rendererLeft2->ResetCamera();
    rendererRight2->SetBackground(.3,.3,.3);
    rendererRight2->AddActor(actorRight2);

    renderWindow2->AddRenderer(rendererLeft2);
    renderWindow2->AddRenderer(rendererRight2);
    qvtkWidget->SetRenderWindow(renderWindow);

    qvtkWidget->update();
    int i=0;
    while(1){
        cout<<i<<endl;
        i++;
    }

   qDebug()<<"ending multi-thread"<<endl;
}


void qualityAssessment::leftButton()
{
   this->show();
}
int toggle=1;
void qualityAssessment::rightButton()
{
   if(toggle){
   cout<<"right"<<endl;

   this->ui->qvtkWidget->lower();
   }
   else
   {
       this->ui->qvtkWidget_2->lower();
   }

   toggle=!toggle;
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
 rendererRight->SetBackground(0.3,0.3,0.3);
 rendererRight->AddActor(actorRight);

 // VTK/Qt wedded
 renderWindow->AddRenderer(rendererLeft);
 renderWindow->AddRenderer(rendererRight);

 this->ui->qvtkWidget->SetRenderWindow(renderWindow);
 this->ui->qvtkWidget->GetRenderWindow()->FullScreenOn();
 this->show();
 // Set up action signals and slots
 connect(this->ui->actionExit, SIGNAL(triggered()), this, SLOT(slotExit()));

 this->ui->qvtkWidget_2->lower();
}

void qualityAssessment::slotExit()
{
 qApp->exit();
}


qualityAssessment* qualityAssess;
int main( int argc, char** argv )
{
  // needed to ensure appropriate OpenGL context is created for VTK rendering.
  QSurfaceFormat::setDefaultFormat(QVTKOpenGLWidget::defaultFormat());

  // QT Stuff
  QApplication app( argc, argv );
  qualityAssess = new qualityAssessment;
//QtConcurrent::run(renderNext,qualityAssess->ui->qvtkWidget_2,renderWindow2,qualityAssess);
//qualityAssess->show();

  return app.exec();
}
