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
vtkSmartPointer<vtkDataSetReader> readerLeft = vtkSmartPointer<vtkDataSetReader>::New();
vtkSmartPointer<vtkDataSetReader> readerRight = vtkSmartPointer<vtkDataSetReader>::New();
vtkSmartPointer<vtkPolyDataMapper> mapperLeft =vtkSmartPointer<vtkPolyDataMapper>::New();
vtkSmartPointer<vtkPolyDataMapper> mapperRight =vtkSmartPointer<vtkPolyDataMapper>::New();
vtkSmartPointer<vtkCamera> sharedCamera = vtkSmartPointer<vtkCamera>::New();

std::vector<std::string> displayOrder;
std::ofstream results;
std::ofstream cameraData;
QElapsedTimer timer;

int stimuliCount=-1;
bool finished=false;

//save the camera of the session
void qualityAssessment::interruptHandler(){
    if(!finished && stimuliCount>0 )
    cameraData<<"stiumli "<<stimuliCount<<" @ "
              <<sharedCamera->GetClippingRange()[0]<<","<<sharedCamera->GetClippingRange()[1]<<"/"
              <<sharedCamera->GetFocalPoint()[0]<<","<<sharedCamera->GetFocalPoint()[1]<<","<<sharedCamera->GetFocalPoint()[2]<<"/"
              <<sharedCamera->GetPosition()[0]<<","<<sharedCamera->GetPosition()[1]<<","<<sharedCamera->GetPosition()[2]<<"/"
              <<sharedCamera->GetViewUp()[0]<<","<<sharedCamera->GetViewUp()[1]<<","<<sharedCamera->GetViewUp()[2]<<"/"
              <<sharedCamera->GetViewAngle()*M_PI/180.0<<"/"
              <<"1920,1080/0,0"<<endl;
}
//enable the next button
void qualityAssessment::Checked(){
    this->ui->nextbutton->setEnabled(true);
}

//move focal point with arrow and reset
void qualityAssessment::keyPressEvent(QKeyEvent* ke)
{
    switch(ke->key()){
        case Qt::Key_Up:
            sharedCamera->SetFocalPoint(sharedCamera->GetFocalPoint()[0],sharedCamera->GetFocalPoint()[1]+(actorLeft->GetYRange()[1]-actorLeft->GetYRange()[0])/100,sharedCamera->GetFocalPoint()[2]);
            sharedCamera->SetPosition(sharedCamera->GetPosition()[0],sharedCamera->GetPosition()[1]+(actorLeft->GetYRange()[1]-actorLeft->GetYRange()[0])/100,sharedCamera->GetPosition()[2]);
            renderWindow->Render();
            break;
        case Qt::Key_Down:
            sharedCamera->SetFocalPoint(sharedCamera->GetFocalPoint()[0],sharedCamera->GetFocalPoint()[1]-(actorLeft->GetYRange()[1]-actorLeft->GetYRange()[0])/100,sharedCamera->GetFocalPoint()[2]);
            sharedCamera->SetPosition(sharedCamera->GetPosition()[0],sharedCamera->GetPosition()[1]-(actorLeft->GetYRange()[1]-actorLeft->GetYRange()[0])/100,sharedCamera->GetPosition()[2]);
            renderWindow->Render();
            break;
        case Qt::Key_R:
            rendererLeft->ResetCamera();
            sharedCamera->SetClippingRange(1.914,3.52034);
            sharedCamera->SetPosition(0,.5,3);
            sharedCamera->SetViewUp(0,1,0);
            renderWindow->Render();
        break;
    }
}
//load the next stimuli
void qualityAssessment::next( )
{
    //save the result
    if(this->ui->leftratio->isChecked()){
        if(stimuliCount>0) results<<"1 ";
        this->ui->leftratio->setAutoExclusive(false);
        this->ui->leftratio->setChecked(false);
        this->ui->leftratio->setAutoExclusive(true);

    }
    else if(this->ui->rightratio->isChecked()){
        if(stimuliCount>0)results<<"2 ";
        this->ui->rightratio->setAutoExclusive(false);
        this->ui->rightratio->setChecked(false);
        this->ui->rightratio->setAutoExclusive(true);

    }
    else if(this->ui->equalratio->isChecked()){
        if(stimuliCount>0) results<<"0 ";
        this->ui->equalratio->setAutoExclusive(false);
        this->ui->equalratio->setChecked(false);
        this->ui->equalratio->setAutoExclusive(true);
    }
    this->ui->nextbutton->setEnabled(false);
    if(stimuliCount>0) results<<timer.elapsed()/1000.0<<endl;

    //end of the experiment
    if(displayOrder.size()==0){
        finished=true;
        this->ui->qvtkWidget->hide();
        this->ui->leftratio->hide();
        this->ui->rightratio->hide();
        this->ui->equalratio->hide();
        this->ui->nextbutton->hide();
        this->ui->label_4->hide();
        this->ui->thanks->show();
    }
    else{
        std::string inputFilenameLeft;
        if(stimuliCount==-1){
            inputFilenameLeft = "soldier_cube.vtk";
        }
        else{
            inputFilenameLeft = displayOrder[0];
            results<<inputFilenameLeft<<" ";
            displayOrder.erase(displayOrder.begin());
        }

        readerLeft->SetFileName ( (std::string("contents/")+inputFilenameLeft).c_str());

        mapperLeft->SetInputConnection(readerLeft->GetOutputPort());
        actorLeft->SetMapper(mapperLeft);
        rendererLeft->ResetCamera();
        rendererLeft->AddActor(actorLeft);
    std::string inputFilenameRight;
        if(stimuliCount==-1){
            inputFilenameRight = "soldier_cube.vtk";
         }
        else{
            inputFilenameRight = displayOrder[0];
            results<<inputFilenameRight<<" ";
            displayOrder.erase(displayOrder.begin());
        }

        readerRight->SetFileName ( (std::string("contents/")+inputFilenameRight).c_str() );

        mapperRight->SetInputConnection(readerRight->GetOutputPort());

        actorRight->SetMapper(mapperRight);
        rendererLeft->ResetCamera();
        sharedCamera->SetClippingRange(1.914,3.52034);
        sharedCamera->SetPosition(0,.5,3);
        sharedCamera->SetViewUp(0,1,0);
        rendererRight->AddActor(actorRight);
        renderWindow->Render();
        stimuliCount++;
        timer.start();
    }
    if(displayOrder.size()==0)
    this->ui->nextbutton->setText("End");
}

// Constructor
qualityAssessment::qualityAssessment()
{
    //initialize ui
    this->ui = new Ui_qualityAssessment;
    this->ui->setupUi(this);
    this->ui->qvtkWidget->setGeometry(0,0,this->geometry().width()-148,this->geometry().height());
    this->ui->thanks->hide();

    connect (this->ui->nextbutton,  SIGNAL (clicked ()), this, SLOT (next()));
    connect (this->ui->leftratio,  SIGNAL (clicked ()), this, SLOT (Checked()));
    connect (this->ui->rightratio,  SIGNAL (clicked ()), this, SLOT (Checked()));
    connect (this->ui->equalratio,  SIGNAL (clicked ()), this, SLOT (Checked()));

    this->ui->nextbutton->setEnabled(false);

    //set up interrupt for camera parameters acquisition
    QTimer *interrupt = new QTimer(this);
    connect(interrupt, SIGNAL(timeout()), this, SLOT(interruptHandler()));
    interrupt->start(35);//period in milisecond 30fps

    //exemple of content
    std::string inputFilenameLeft = "contents/soldier_cube.vtk";
    std::string inputFilenameRight = "contents/soldier_disk_sc2.vtk";

    readerLeft->SetFileName ( inputFilenameLeft.c_str() );
    readerRight->SetFileName ( inputFilenameRight.c_str() );

    // Visualize
    vtkSmartPointer<vtkPolyDataMapper> mapperLeft =vtkSmartPointer<vtkPolyDataMapper>::New();

    mapperLeft->SetInputConnection(readerLeft->GetOutputPort());


    vtkSmartPointer<vtkPolyDataMapper> mapperRight =vtkSmartPointer<vtkPolyDataMapper>::New();

    mapperRight->SetInputConnection(readerRight->GetOutputPort());

    actorLeft->SetMapper(mapperLeft);
    actorRight->SetMapper(mapperRight);

    actorLeft->GetProperty()->LightingOff();
    actorRight->GetProperty()->LightingOff();

    // VTK Renderer

    rendererLeft->SetViewport(0,0,0.5,1);
    rendererLeft->SetActiveCamera(sharedCamera);
    rendererLeft->AddActor(actorLeft);
    rendererLeft->ResetCamera();

    rendererRight->SetViewport(0.5,0,1,1);
    rendererRight->SetActiveCamera(sharedCamera);
    rendererRight->AddActor(actorRight);
    //sharedCamera->SetParallelProjection(0);

    renderWindow->AddRenderer(rendererLeft);
    renderWindow->AddRenderer(rendererRight);

    this->ui->qvtkWidget->SetRenderWindow(renderWindow);
    this->ui->qvtkWidget->GetRenderWindow()->FullScreenOn();

    timer.start();
}

void qualityAssessment::slotExit()
{
    qApp->exit();
}

int main( int argc, char** argv )
{
  std::string inputFile= argv[1];
  std::ifstream orderFile(inputFile);

  //remove the folder name
  inputFile.erase(0,8);
  //create and open the save files
  results.open(std::string("results/result_")+inputFile, std::ios_base::out | std::ios_base::out);
  cameraData.open(std::string("camera/")+inputFile.replace(inputFile.size()-3,inputFile.size(),"cam"), std::ios_base::out | std::ios_base::out);

  //read the content of the batch file
  std::string leftContent,rightContent;
  while (orderFile >> leftContent >> rightContent){
    displayOrder.push_back(leftContent);
    displayOrder.push_back(rightContent);
  }

  // needed to ensure appropriate OpenGL context is created for VTK rendering.
  QSurfaceFormat::setDefaultFormat(QVTKOpenGLWidget::defaultFormat());

  QApplication app( argc, argv );
  qualityAssessment  qualityAssessment;
//  qualityAssessment.show();
  qualityAssessment.showFullScreen();

  return app.exec();
}
