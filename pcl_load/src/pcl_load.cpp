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

int start_time,stop_time;

enum representationType{SQUARE,DISK,CUBE,SPHERE};
enum rendererWindow{left,right};

int start,end;
vtkSmartPointer<vtkActor> Surface2D(std::string &filename,int PolygonType)
{
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	std::ifstream filestream(filename.c_str());
	vtkSmartPointer<vtkDoubleArray> tensors = vtkSmartPointer<vtkDoubleArray>::New();
	tensors->SetNumberOfTuples(3);
	tensors->SetNumberOfComponents(9);
	std::string line;
	//skip the infos
	std::getline(filestream, line);
	std::getline(filestream, line);
	//extract the points values
	start=clock();
	while(std::getline(filestream, line))
	{
	  double x, y, z,r,g,b,rotation[9];

	  std::stringstream linestream;
	  linestream << line;
	  linestream >> x >> y >> z>>r>>g>>b>>rotation[0]>>rotation[1]>>rotation[2]>>rotation[3]>>rotation[4]>>rotation[5]>>rotation[6]>>rotation[7]>>rotation[8];
	  points->InsertNextPoint(x, y, z);
	  tensors->InsertNextTuple9(rotation[0],rotation[1],rotation[2],rotation[3],rotation[4],rotation[5],rotation[6],rotation[7],rotation[8]);
	}
	filestream.close();
	end=clock();
	cout << "\nExec time: " << (end-start)/double(CLOCKS_PER_SEC)<< " s "<< endl;
	vtkSmartPointer<vtkPolyData> polydata =  vtkSmartPointer<vtkPolyData>::New();
	polydata->SetPoints(points);
	polydata->GetPointData()->SetTensors(tensors);

	// Create a circle
	vtkSmartPointer<vtkRegularPolygonSource> polygonSource =  vtkSmartPointer<vtkRegularPolygonSource>::New();

	if(PolygonType==DISK)polygonSource->SetNumberOfSides(20);
	else if(PolygonType==SQUARE)polygonSource->SetNumberOfSides(4);
	polygonSource->SetRadius(1);
	polygonSource->GeneratePolylineOff();

	polygonSource->Update();

	vtkSmartPointer<vtkTensorGlyph> tensorGlyph = vtkSmartPointer<vtkTensorGlyph>::New();
	tensorGlyph->SetInputData(polydata);
	tensorGlyph->SetSourceConnection(polygonSource->GetOutputPort());
	tensorGlyph->ColorGlyphsOff();
	tensorGlyph->ThreeGlyphsOff();
	tensorGlyph->ExtractEigenvaluesOff();

	tensorGlyph->SymmetricOff();
	tensorGlyph->Update();

	// Visualize

	vtkSmartPointer<vtkOpenGLPolyDataMapper> leftMapper =  vtkSmartPointer<vtkOpenGLPolyDataMapper>::New();
	leftMapper->SetInputData(tensorGlyph->GetOutput());

	vtkSmartPointer<vtkActor> leftActor =  vtkSmartPointer<vtkActor>::New();
	leftActor->SetMapper(leftMapper);

	return leftActor;
}


int main(int argc, char ** argv)
{
	std::string line;

	start_time=clock();

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
	std::getline(leftFile, line);
	std::size_t found = line.find("type: disk");
	if (found!=std::string::npos)
	{
		cout<<"representation: disks\n";
		leftFile.close();
		leftActor=Surface2D(cloud_left,DISK);
	}
	found = line.find("type: square");
	if (found!=std::string::npos)
		{
			cout<<"representation: square\n";
			leftFile.close();
			leftActor=Surface2D(cloud_left,SQUARE);
		}

	//load the second actor
	std::string cloud_right(argv[2]);
	std::ifstream rightFile(cloud_right.c_str());
	vtkSmartPointer<vtkActor> rightActor =  vtkSmartPointer<vtkActor>::New();


	//surface type
	std::getline(rightFile, line);
	found = line.find("type: disk");
	if (found!=std::string::npos)
	{
		cout<<"representation: disk\n";
		rightActor=Surface2D(cloud_right,DISK);
		rightFile.close();
	}
	found = line.find("type: square");
	if (found!=std::string::npos)
	{
		cout<<"representation: square\n";
		rightActor=Surface2D(cloud_right,SQUARE);
		rightFile.close();
	}

	// There will be one render window
	vtkSmartPointer<vtkRenderWindow> renderWindow =  vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->SetSize(1800, 900);

	// And one interactor
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	interactor->SetRenderWindow(renderWindow);

	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New(); //like paraview
	 interactor->SetInteractorStyle(style);

	// Define viewport ranges
	double leftViewport[4] = {0.0, 0.0, 0.5, 1.0};
	double rightViewport[4] = {0.5, 0.0, 1.0, 1.0};

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

	// Add the sphere to the left and the cube to the right
	leftRenderer->AddActor(leftActor);
	rightRenderer->AddActor(rightActor);

	leftRenderer->ResetCamera();
	rightRenderer->ResetCamera();

	renderWindow->Render();
	stop_time=clock();
	cout << "\nExec time: " << (stop_time-start_time)/double(CLOCKS_PER_SEC)<< " s "<< endl;
	interactor->Start();

	return EXIT_SUCCESS;
}
