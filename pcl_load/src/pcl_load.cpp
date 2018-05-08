#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkTensorGlyph.h>
#include <vtkOpenGLPolyDataMapper.h>
#include <vtkDoubleArray.h>
#include <vtkContourFilter.h>
#include <vtkGaussianSplatter.h>
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

enum representationType{SQUARE,DISK,CUBE,SPHERE};

// There will be one render window
	vtkSmartPointer<vtkRenderWindow> renderWindow =  vtkSmartPointer<vtkRenderWindow>::New();

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
      std::cout << "Pressed " << key << std::endl;

      // Handle an arrow key
      if(key == "Return")
        {
    	  renderWindow->Render();


        }

      // Handle a "normal" key
      if(key == "a")
        {
        std::cout << "The a key was pressed." << std::endl;
        }

      // Forward events
      vtkInteractorStyleTrackballCamera::OnKeyPress();
    }

};
vtkStandardNewMacro(KeyPressInteractorStyle);
vtkSmartPointer<vtkActor> Splats2D(std::string &filename,int PolygonType)
{
	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
	reader->SetFileName(filename.c_str());
	reader->Update();

	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(reader->GetOutputPort());

	vtkSmartPointer<vtkActor> actor =  vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	return actor;
}

vtkSmartPointer<vtkActor> polygon3D(std::string &filename,int PolygonType)
{
	std::ifstream filestream(filename.c_str());
	// Create points
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

	// setup scales
	vtkSmartPointer<vtkFloatArray> scales = vtkSmartPointer<vtkFloatArray>::New();
	scales->SetName("scales");

	vtkSmartPointer<vtkFloatArray> col = vtkSmartPointer<vtkFloatArray>::New();
	col->SetName("col");

	vtkSmartPointer<vtkLookupTable> lut = vtkSmartPointer<vtkLookupTable>::New();

	std::string line;
	//skip the infos

	std::getline(filestream, line);//type
	std::getline(filestream, line);//numb_point

	std::string numbString = line.erase(0,7);
	int numberPoints= atoi(numbString.c_str());
	printf("number of cubes: %d \n",numberPoints);
	lut->SetNumberOfTableValues(numberPoints);

	std::getline(filestream, line);//format

	//extract the points values
	int i=0;
	while(std::getline(filestream, line))
	{
	  double x, y, z,r,g,b,scale;

	  std::stringstream linestream;
	  linestream << line;
	  linestream >> x >> y >> z>>r>>g>>b>>scale;
	  points->InsertNextPoint(x, y, z);
	  scales->InsertNextValue(scale);
	  col->InsertNextValue(i);
	  lut->SetTableValue(i,r,g,b);
	  i++;
	}
	filestream.close();
	// grid structured to append center, radius and color label
	vtkSmartPointer<vtkUnstructuredGrid> grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
	grid->SetPoints(points);
	grid->GetPointData()->AddArray(scales);
	grid->GetPointData()->SetActiveScalars("scales"); // !!!to set radius first
	grid->GetPointData()->AddArray(col);


	vtkSmartPointer<vtkCubeSource> cubeSource =  vtkSmartPointer<vtkCubeSource>::New();
	vtkSmartPointer<vtkSphereSource> sphereSource =  vtkSmartPointer<vtkSphereSource>::New();
	sphereSource->SetRadius(.57);
	vtkSmartPointer<vtkGlyph3D> glyph3D = vtkSmartPointer<vtkGlyph3D>::New();
	glyph3D->SetInputData(grid);
	if(PolygonType==CUBE)
		glyph3D->SetSourceConnection(cubeSource->GetOutputPort());
	else
		glyph3D->SetSourceConnection(sphereSource->GetOutputPort());



	// Create a mapper
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(glyph3D->GetOutputPort());
	mapper->SetScalarModeToUsePointFieldData();
	mapper->SetScalarRange(0,numberPoints);
	mapper->SelectColorArray("col");
	mapper->SetLookupTable(lut);

	vtkSmartPointer<vtkActor> actor =  vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);


	return actor;
}


int main(int argc, char ** argv)
{

	std::string line;

	if(argc!=3 )
	{
		printf("./surface_recon cloudpath1 cloudpath2  \n");
		printf("exemple: ./surface_recon bunny_disk.txt bunny_square.txt\n");
		return EXIT_FAILURE;
	}

	//load the first actor
	std::string cloud_left(argv[1]);
	std::ifstream leftFile(cloud_left.c_str());

	vtkSmartPointer<vtkActor> leftActor =  vtkSmartPointer<vtkActor>::New();

	//surface type

	std::size_t found = cloud_left.find("disk");
	if (found!=std::string::npos)
	{
		cout<<"representation: disks\n";
		leftFile.close();
		leftActor=Splats2D(cloud_left,DISK);
	}
	found = cloud_left.find("square");
	if (found!=std::string::npos)
	{
		cout<<"representation: square\n";
		leftFile.close();
		leftActor=Splats2D(cloud_left,SQUARE);
		}
	found = cloud_left.find("cube");
	if (found!=std::string::npos)
	{
		cout<<"representation: cube\n";
		leftFile.close();
		leftActor=polygon3D(cloud_left,CUBE);
	}
	found = cloud_left.find("sphere");
	if (found!=std::string::npos)
	{
		cout<<"representation: sphere\n";
		leftFile.close();
		leftActor=polygon3D(cloud_left,SPHERE);
	}
//	cl = clock() - cl;  //end point of clock
//		cout<<"first actor time: "<<cl/1000000.0<< "s"<<endl;
	//load the second actor
	std::string cloud_right(argv[2]);
	std::ifstream rightFile(cloud_right.c_str());
	vtkSmartPointer<vtkActor> rightActor =  vtkSmartPointer<vtkActor>::New();


	//surface type
	std::getline(rightFile, line);
	found = cloud_right.find("disk");
	if (found!=std::string::npos)
	{
		cout<<"representation: disk\n";
		rightActor=Splats2D(cloud_right,DISK);
		rightFile.close();
	}
	found = cloud_right.find("square");
	if (found!=std::string::npos)
	{
		cout<<"representation: square\n";
		rightActor=Splats2D(cloud_right,SQUARE);
		rightFile.close();
	}
	found = cloud_right.find("cube");
	if (found!=std::string::npos)
	{
		cout<<"representation: cube\n";
		rightFile.close();
		rightActor=polygon3D(cloud_right,CUBE);
	}
	found = cloud_right.find("sphere");
	if (found!=std::string::npos)
	{
		cout<<"representation: sphere\n";
		rightFile.close();
		rightActor=polygon3D(cloud_right,SPHERE);
	}
	renderWindow->SetSize(1920,1080);


	// And one interactor
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	interactor->SetRenderWindow(renderWindow);

//	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New(); //like paraview
//	 interactor->SetInteractorStyle(style);

	// Define viewport ranges
	double leftViewport[4] = {0.0, 0.2, 0.5, 1};
	double rightViewport[4] = {0.5, 0.2, 1.0, 1};
	double downViewportleft[4] = {0.0, 0.0, .3, 0.2};
	double downViewportmiddle[4] = {0.3, 0.0, .7, 0.2};
	double downViewportright[4] = {0.7, 0.0, 1.0, 0.2};

	// Shared camera
	vtkSmartPointer<vtkCamera> sharedCamera = vtkSmartPointer<vtkCamera>::New();

	// Setup both renderers
	vtkSmartPointer<vtkRenderer> leftRenderer = vtkSmartPointer<vtkRenderer>::New();
	renderWindow->AddRenderer(leftRenderer);
	leftRenderer->SetViewport(leftViewport);
	leftRenderer->SetActiveCamera(sharedCamera);

	vtkSmartPointer<vtkRenderer> rightRenderer = vtkSmartPointer<vtkRenderer>::New();
	renderWindow->AddRenderer(rightRenderer);
	rightRenderer->SetViewport(rightViewport);
	rightRenderer->SetActiveCamera(sharedCamera);

	vtkSmartPointer<vtkRenderer> downRendererleft = vtkSmartPointer<vtkRenderer>::New();
	renderWindow->AddRenderer(downRendererleft);
	downRendererleft->SetViewport(downViewportleft);
	vtkSmartPointer<vtkRenderer> downRenderermiddle = vtkSmartPointer<vtkRenderer>::New();
	renderWindow->AddRenderer(downRenderermiddle);
	downRenderermiddle->SetViewport(downViewportmiddle);
	downRenderermiddle->SetBackground(0.3,0.3,0.3);
	vtkSmartPointer<vtkRenderer> downRendererright = vtkSmartPointer<vtkRenderer>::New();
	renderWindow->AddRenderer(downRendererright);
	downRendererright->SetViewport(downViewportright);
	// Setup the text and add it to the renderer

	vtkSmartPointer<vtkTextActor> textActorleft = vtkSmartPointer<vtkTextActor>::New();
	vtkSmartPointer<vtkTextActor> textActormiddle = vtkSmartPointer<vtkTextActor>::New();
	vtkSmartPointer<vtkTextActor> textActorright = vtkSmartPointer<vtkTextActor>::New();

	textActorleft->SetInput ( "<=" );
	textActorleft->SetDisplayPosition(downRendererleft->GetCenter()[0]-60,50);
	textActorleft->GetTextProperty()->SetFontSize ( 120 );
	textActorleft->GetTextProperty()->SetColor ( 1.0, 1.0, 1.0 );
	downRendererleft->AddActor2D ( textActorleft );

	textActormiddle->SetInput ( "Enter to start" );
	textActormiddle->SetDisplayPosition(downRenderermiddle->GetCenter()[0]-100,80);
	textActormiddle->GetTextProperty()->SetFontSize ( 40 );
	textActormiddle->GetTextProperty()->SetColor ( 1.0, 1.0, 1.0 );
	downRenderermiddle->AddActor2D ( textActormiddle );

	textActorright->SetInput ( "=>" );
	textActorright->SetDisplayPosition(downRendererright->GetCenter()[0]-50,50);
	textActorright->GetTextProperty()->SetFontSize ( 120 );
	textActorright->GetTextProperty()->SetColor ( 1.0, 1.0, 1.0 );
	downRendererright->AddActor2D ( textActorright );

	//remove light
	leftActor->GetProperty()->LightingOff();
	rightActor->GetProperty()->LightingOff();
	renderWindow->Render();
	// Add the actors
	leftRenderer->AddActor(leftActor);
	rightRenderer->AddActor(rightActor);
	printf("ready\n");
	vtkSmartPointer<KeyPressInteractorStyle> style = vtkSmartPointer<KeyPressInteractorStyle>::New();
	interactor->SetInteractorStyle(style);
	style->SetCurrentRenderer(leftRenderer);

	leftRenderer->ResetCamera();
	rightRenderer->ResetCamera();


	interactor->Start();

	return EXIT_SUCCESS;
}
