#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkTensorGlyph.h>
#include <vtkOpenGLPolyDataMapper.h>
#include <vtkDoubleArray.h>
#include <vtkRegularPolygonSource.h>
#include <vtkReverseSense.h>
#include <vtkSurfaceReconstructionFilter.h>
#include <vtkDelaunay3D.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <limits>
#include <vtkCubeSource.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkPointData.h>
#include <vtkFloatArray.h>
#include <vtkUnstructuredGrid.h>
#include <vtkLookupTable.h>
#include <vtkGlyph3D.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>
#include <vtkPLYReader.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkArrowSource.h>
#include <algorithm>

bool next=false;

#define NB_PCL 2
#define NB_REP 4
std::string files[NB_PCL][NB_REP]={{"andrew9_cube.ply","andrew9_disk.ply","andrew9_sphere.ply","andrew9_square.ply"},
						 {"mask_cube.ply","mask_disk.ply","mask_sphere.ply","mask_square.ply"},
	};
enum representationType{SQUARE,DISK,CUBE,SPHERE};
std::vector< int > random_list;

vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
vtkSmartPointer<vtkRenderer> rendererLeft = vtkSmartPointer<vtkRenderer>::New();
vtkSmartPointer<vtkRenderer> rendererRight = vtkSmartPointer<vtkRenderer>::New();
vtkSmartPointer<vtkRenderer> rendererDown = vtkSmartPointer<vtkRenderer>::New();
vtkSmartPointer<vtkActor> actorLeft =  vtkSmartPointer<vtkActor>::New();
vtkSmartPointer<vtkActor> actorRight =  vtkSmartPointer<vtkActor>::New();
vtkSmartPointer<vtkTextActor> textActorWait = vtkSmartPointer<vtkTextActor>::New();
vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();

void nextActor(){

	vtkSmartPointer<vtkPLYReader> readerLeft = vtkSmartPointer<vtkPLYReader>::New();
	vtkSmartPointer<vtkPLYReader> readerRight = vtkSmartPointer<vtkPLYReader>::New();
	int next_pcl=random_list.back()/100;
	int rep_left=(random_list.back()-100*next_pcl)/10;
	int rep_right=random_list.back()-100*next_pcl-10*rep_left;
	printf("choice: %d %d %d ",next_pcl,rep_left,rep_right);
	random_list.pop_back();

	readerLeft->SetFileName ( files[next_pcl][rep_left].c_str() );
	// Visualize

	vtkSmartPointer<vtkPolyDataMapper> mapperLeft = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapperLeft->SetInputConnection(readerLeft->GetOutputPort());
	actorLeft->SetMapper(mapperLeft);

	vtkSmartPointer<vtkPolyDataMapper> mapperRight = vtkSmartPointer<vtkPolyDataMapper>::New();
	readerRight->SetFileName ( files[next_pcl][rep_right].c_str() );
	mapperRight->SetInputConnection(readerRight->GetOutputPort());
	actorRight->SetMapper(mapperRight);

	actorLeft->GetProperty()->LightingOff();
	actorRight->GetProperty()->LightingOff();

	renderWindow->Render();
	rendererLeft->ResetCamera();
}

class update : public vtkCommand
{
public:
    vtkTypeMacro(update, vtkCommand);

    static update * New()
    {
        return new update;
    }

    void Execute(vtkObject * vtkNotUsed(caller),
                 unsigned long vtkNotUsed(eventId),
                 void * vtkNotUsed(callData))
    {
    	renderWindow->Render();
    	if(next){
    		renderWindowInteractor->Disable();
    		textActorWait->SetInput("Wait!");
    		textActorWait->SetVisibility(1);
    		renderWindow->Render();
    		next=false;
    		nextActor();
    		textActorWait->SetVisibility(0);

    		renderWindow->Render();
    		renderWindowInteractor->Enable();
    	}
    }
};

// Define interaction style
class KeyPressInteractorStyle : public vtkInteractorStyleTrackballCamera
{
  public:
    static KeyPressInteractorStyle* New();
    vtkTypeMacro(KeyPressInteractorStyle, vtkInteractorStyleTrackballCamera);

    virtual void OnKeyPress()
    {
      // Get the keypress
      vtkRenderWindowInteractor *rwi = this->Interactor;
      std::string key = rwi->GetKeySym();

      // Output the key that was pressed
//      std::cout << "Pressed " << key << std::endl;

      // Handle an arrow key
      if(key == "Right")
        {

    	  next=true;
    	  printf("chosen: right\n");
        }
      if(key == "Left")
      {
    	  next=true;
    	  printf("chosen: left\n");
      }
      // Forward events
      vtkInteractorStyleTrackballCamera::OnKeyPress();
    }

};
vtkStandardNewMacro(KeyPressInteractorStyle);


int main(int argc, char ** argv)
{
	srand (time(NULL));
	std::string line;

	for(int i=0;i<NB_PCL;i++)
	{
		for(int j=0;j<NB_REP;j++){
			for(int k=j+1;k<NB_REP;k++)
			random_list.push_back(100*i+10*j+k);
		}
	}
	std::random_shuffle(random_list.begin(), random_list.end());
	random_list.push_back(13);
	random_list.push_back(23);
	for(int i=0;i<random_list.size();i++){
		printf("%d ",random_list[i]);
	}
	printf("\n");
	int pcl,representation,representation2;

	vtkSmartPointer<vtkPLYReader> readerLeft = vtkSmartPointer<vtkPLYReader>::New();
	vtkSmartPointer<vtkPLYReader> readerRight = vtkSmartPointer<vtkPLYReader>::New();
	readerLeft->SetFileName ( "bunny_cube.ply" );
	printf("exemple press left or right \n");
	// Visualize
	vtkSmartPointer<vtkPolyDataMapper> mapperLeft = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapperLeft->SetInputConnection(readerLeft->GetOutputPort());
	actorLeft->SetMapper(mapperLeft);
	readerRight->SetFileName("bunny_disk.ply");
	vtkSmartPointer<vtkPolyDataMapper> mapperRight = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapperRight->SetInputConnection(readerRight->GetOutputPort());
	actorRight->SetMapper(mapperRight);


	renderWindow->AddRenderer(rendererLeft);
	renderWindow->AddRenderer(rendererRight);
	renderWindow->AddRenderer(rendererDown);


	renderWindowInteractor->SetRenderWindow(renderWindow);

	vtkSmartPointer<KeyPressInteractorStyle> style =  vtkSmartPointer<KeyPressInteractorStyle>::New();
	style->SetInteractor(renderWindowInteractor);
	renderWindowInteractor->SetInteractorStyle(style);
	style->SetCurrentRenderer(rendererLeft);

	vtkSmartPointer<vtkCamera> sharedCamera = vtkSmartPointer<vtkCamera>::New();

actorLeft->GetProperty()->SetInterpolationToFlat();
actorRight->GetProperty()->SetInterpolationToPhong();
	rendererLeft->AddActor(actorLeft);
	rendererLeft->SetViewport(0,0.2,0.5,1);
	rendererLeft->SetActiveCamera(sharedCamera);
	rendererRight->AddActor(actorRight);
	rendererRight->SetViewport(0.5,0.2,1,1);
	rendererRight->SetActiveCamera(sharedCamera);
	rendererLeft->ResetCamera();
	rendererDown->SetViewport(0,0,1,.2);

	renderWindow->SetSize(1360,800);
//	renderWindow->FullScreenOn();
	renderWindow->Render();
	//--------------------------------------------------------------






	//--------------------------------------------------------
	// Setup the text and add it to the renderer
	vtkSmartPointer<vtkTextActor> textActorLeft = vtkSmartPointer<vtkTextActor>::New();
	textActorLeft->SetInput ( "<" );
	textActorLeft->SetPosition( rendererLeft->GetCenter()[0]-100, -10 );
	textActorLeft->GetTextProperty()->SetFontSize ( 180 );
	textActorLeft->GetTextProperty()->SetColor ( 1.0, 1.0, 1.0 );
	rendererDown->AddActor2D ( textActorLeft );
	vtkSmartPointer<vtkTextActor> textActorRight = vtkSmartPointer<vtkTextActor>::New();
	textActorRight->SetInput ( ">" );
	textActorRight->SetPosition( rendererRight->GetCenter()[0]-50, -10 );
	textActorRight->GetTextProperty()->SetFontSize ( 180 );
	textActorRight->GetTextProperty()->SetColor ( 1.0, 1.0, 1.0 );
	rendererDown->AddActor2D ( textActorRight );

	textActorWait->SetInput ( "Choose" );

	textActorWait->SetPosition( rendererDown->GetCenter()[0]-80, 50 );
	textActorWait->GetTextProperty()->SetFontSize ( 50 );
	textActorWait->GetTextProperty()->SetColor ( 1.0, 1.0, 1.0 );
	rendererDown->AddActor2D ( textActorWait );

	renderWindow->Render();
	renderWindowInteractor->CreateRepeatingTimer(1);
	update * Callback =  update::New();
	renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, Callback );
	renderWindowInteractor->Start();


	return EXIT_SUCCESS;
}
