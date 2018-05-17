#include "qualityAssessment.h"

// This is included here because it is forward declared in
// RenderWindowUISingleInheritance.h
#include "ui_qualityAssessment.h"
#include "../build/ui_qualityAssessment.h"

vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow=vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
vtkSmartPointer<vtkRenderer> rendererLeft=vtkSmartPointer<vtkRenderer>::New();
vtkSmartPointer<vtkRenderer> rendererRight=vtkSmartPointer<vtkRenderer>::New();

vtkSmartPointer<vtkActor> actorLeft =vtkSmartPointer<vtkActor>::New();
vtkSmartPointer<vtkActor> actorRight =vtkSmartPointer<vtkActor>::New();
vtkSmartPointer<vtkPLYReader> readerLeft = vtkSmartPointer<vtkPLYReader>::New();
vtkSmartPointer<vtkPLYReader> readerRight = vtkSmartPointer<vtkPLYReader>::New();
vtkSmartPointer<vtkPolyDataMapper> mapperLeft =vtkSmartPointer<vtkPolyDataMapper>::New();
vtkSmartPointer<vtkPolyDataMapper> mapperRight =vtkSmartPointer<vtkPolyDataMapper>::New();

std::vector<std::string> displayOrder;
void qualityAssessment::next()
{
    std::string inputFilenameLeft = displayOrder[0];
    cout<<inputFilenameLeft<<endl;
    displayOrder.erase(displayOrder.begin());
     readerLeft->SetFileName ( inputFilenameLeft.c_str() );
     vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
     if(inputFilenameLeft.find('_') != std::string::npos){
     mapperLeft->SetInputConnection(readerLeft->GetOutputPort());
     }
     else{
         vertexGlyphFilter->SetInputConnection(readerLeft->GetOutputPort());
         vertexGlyphFilter->Update();
          mapperLeft->SetInputConnection(vertexGlyphFilter->GetOutputPort());
     }
     actorLeft->SetMapper(mapperLeft);
     rendererLeft->ResetCamera();
     rendererLeft->AddActor(actorLeft);

     std::string inputFilenameRight = displayOrder[0];
     displayOrder.erase(displayOrder.begin());
      readerRight->SetFileName ( inputFilenameRight.c_str() );
    cout<<inputFilenameRight<<endl;
      if(inputFilenameRight.find('_') != std::string::npos){
      mapperRight->SetInputConnection(readerRight->GetOutputPort());
      }
      else{
          vertexGlyphFilter->SetInputConnection(readerRight->GetOutputPort());
          vertexGlyphFilter->Update();
           mapperRight->SetInputConnection(vertexGlyphFilter->GetOutputPort());
      }
      actorRight->SetMapper(mapperRight);
      rendererRight->ResetCamera();
      rendererRight->AddActor(actorRight);
}

// Constructor
qualityAssessment::qualityAssessment()
{
    this->ui = new Ui_qualityAssessment;
    this->ui->setupUi(this);

    std::string inputFilenameLeft = "content/longdress.ply";
    std::string inputFilenameRight = "content/longdress.ply";


    vtkSmartPointer<vtkPLYReader> readerLeft = vtkSmartPointer<vtkPLYReader>::New();
    readerLeft->SetFileName ( inputFilenameLeft.c_str() );
    vtkSmartPointer<vtkPLYReader> readerRight = vtkSmartPointer<vtkPLYReader>::New();
    readerRight->SetFileName ( inputFilenameRight.c_str() );

    connect (this->ui->nextbutton,  SIGNAL (clicked ()), this, SLOT (next()));


    vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();


    // Visualize
    vtkSmartPointer<vtkPolyDataMapper> mapperLeft =vtkSmartPointer<vtkPolyDataMapper>::New();
    if(inputFilenameLeft.find('_') != std::string::npos){
    mapperLeft->SetInputConnection(readerLeft->GetOutputPort());
    }
    else{
        vertexGlyphFilter->SetInputConnection(readerLeft->GetOutputPort());
        vertexGlyphFilter->Update();
         mapperLeft->SetInputConnection(vertexGlyphFilter->GetOutputPort());
    }



    vtkSmartPointer<vtkPolyDataMapper> mapperRight =vtkSmartPointer<vtkPolyDataMapper>::New();
    if(inputFilenameRight.find('_') != std::string::npos){
    mapperRight->SetInputConnection(readerRight->GetOutputPort());
    }
    else{
        vertexGlyphFilter->SetInputConnection(readerRight->GetOutputPort());
        vertexGlyphFilter->Update();
         mapperRight->SetInputConnection(vertexGlyphFilter->GetOutputPort());
    }

    actorLeft->SetMapper(mapperLeft);
    actorRight->SetMapper(mapperRight);

    actorLeft->GetProperty()->LightingOff();
    actorRight->GetProperty()->LightingOff();

    vtkSmartPointer<vtkCamera> sharedCamera = vtkSmartPointer<vtkCamera>::New();

    //actor->GetProperty()->LightingOff();
    // VTK Renderer

    rendererLeft->SetViewport(0,0,0.5,1);
    rendererLeft->SetActiveCamera(sharedCamera);
    rendererLeft->AddActor(actorLeft);
    rendererLeft->ResetCamera();

    rendererRight->SetViewport(0.5,0,1,1);
    rendererRight->SetActiveCamera(sharedCamera);

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

}

void qualityAssessment::slotExit()
{
 qApp->exit();
}

int main( int argc, char** argv )
{
  std::ifstream orderFile(argv[1]);
  std::string leftContent,rightContent;
  while (orderFile >> leftContent >> rightContent){
    displayOrder.push_back(leftContent);
    displayOrder.push_back(rightContent);
  }



  // needed to ensure appropriate OpenGL context is created for VTK rendering.
  QSurfaceFormat::setDefaultFormat(QVTKOpenGLWidget::defaultFormat());

  // QT Stuff
  QApplication app( argc, argv );
  qualityAssessment  qualityAssessment;
  qualityAssessment.show();

  return app.exec();
}
