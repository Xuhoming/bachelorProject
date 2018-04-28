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
int start_time,stop_time;

enum representationType{SQUARE,DISK,CUBE,SPHERE};
enum rendererWindow{left,right};

int start,end;
vtkSmartPointer<vtkActor> Splats2D(std::string &filename,int PolygonType)
{
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	std::ifstream filestream(filename.c_str());
	vtkSmartPointer<vtkDoubleArray> tensors = vtkSmartPointer<vtkDoubleArray>::New();
	tensors->SetNumberOfTuples(3);
	tensors->SetNumberOfComponents(9);

	vtkSmartPointer<vtkFloatArray> col = vtkSmartPointer<vtkFloatArray>::New();
	col->SetName("col");

	vtkSmartPointer<vtkLookupTable> lut = vtkSmartPointer<vtkLookupTable>::New();

	std::string line;
	//skip the infos

	std::getline(filestream, line);//type
	std::getline(filestream, line);//numb_point

	std::string numbString = line.erase(0,7);
	int numberPoints= atoi(numbString.c_str());

	lut->SetNumberOfTableValues(numberPoints);

	printf("#of splats: %d \n",numberPoints);

	std::getline(filestream, line);//format

	//extract the points values
	start=clock();
	int i=0;
	while(std::getline(filestream, line))
	{
	  double x, y, z,r,g,b,rotation[9];

	  std::stringstream linestream;
	  linestream << line;
	  linestream >> x >> y >> z>>r>>g>>b>>rotation[0]>>rotation[1]>>rotation[2]>>rotation[3]>>rotation[4]>>rotation[5]>>rotation[6]>>rotation[7]>>rotation[8];
	  points->InsertNextPoint(x, y, z);
	  tensors->InsertNextTuple9(rotation[0],rotation[1],rotation[2],rotation[3],rotation[4],rotation[5],rotation[6],rotation[7],rotation[8]);
	  col->InsertNextValue(i);
	  lut->SetTableValue(i,r,g,b);
	  i++;
	}
	filestream.close();
	end=clock();
	cout << "\nExec time: " << (end-start)/double(CLOCKS_PER_SEC)<< " s "<< endl;
	vtkSmartPointer<vtkUnstructuredGrid> grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
	grid->SetPoints(points);
	grid->GetPointData()->SetTensors(tensors);
	grid->GetPointData()->AddArray(col);
	grid->GetPointData()->SetActiveScalars("col");;

	// Create a circle
	vtkSmartPointer<vtkRegularPolygonSource> polygonSource =  vtkSmartPointer<vtkRegularPolygonSource>::New();

	if(PolygonType==DISK)polygonSource->SetNumberOfSides(20);
	else if(PolygonType==SQUARE)polygonSource->SetNumberOfSides(4);
	polygonSource->SetRadius(1);
	polygonSource->GeneratePolylineOff();
	polygonSource->Update();

	vtkSmartPointer<vtkTensorGlyph> tensorGlyph = vtkSmartPointer<vtkTensorGlyph>::New();
	tensorGlyph->SetInputData(grid);
	tensorGlyph->SetSourceConnection(polygonSource->GetOutputPort());
	tensorGlyph->ColorGlyphsOn();
	tensorGlyph->ThreeGlyphsOff();
	tensorGlyph->SetColorModeToScalars();
	tensorGlyph->ExtractEigenvaluesOff();

	tensorGlyph->SymmetricOff();
	tensorGlyph->Update();

	// Visualize

	vtkSmartPointer<vtkOpenGLPolyDataMapper> mapper =  vtkSmartPointer<vtkOpenGLPolyDataMapper>::New();
	mapper->SetInputData(tensorGlyph->GetOutput());
	mapper->SetInputConnection(tensorGlyph->GetOutputPort());
	mapper->SetScalarModeToUsePointFieldData();
	mapper->SetScalarRange(0,numberPoints);
	mapper->SelectColorArray("col");
	mapper->SetLookupTable(lut);

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
	start=clock();
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
	sphereSource->SetRadius(.6);
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
		leftActor=Splats2D(cloud_left,DISK);
	}
	found = line.find("type: square");
	if (found!=std::string::npos)
	{
		cout<<"representation: square\n";
		leftFile.close();
		leftActor=Splats2D(cloud_left,SQUARE);
		}
	found = line.find("type: cube");
	if (found!=std::string::npos)
	{
		cout<<"representation: cube\n";
		leftFile.close();
		leftActor=polygon3D(cloud_left,CUBE);
	}
	found = line.find("type: sphere");
	if (found!=std::string::npos)
	{
		cout<<"representation: sphere\n";
		leftFile.close();
		leftActor=polygon3D(cloud_left,SPHERE);
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
		rightActor=Splats2D(cloud_right,DISK);
		rightFile.close();
	}
	found = line.find("type: square");
	if (found!=std::string::npos)
	{
		cout<<"representation: square\n";
		rightActor=Splats2D(cloud_right,SQUARE);
		rightFile.close();
	}
	found = line.find("type: cube");
	if (found!=std::string::npos)
	{
		cout<<"representation: cube\n";
		rightFile.close();
		rightActor=polygon3D(cloud_right,CUBE);
	}
	found = line.find("type: sphere");
	if (found!=std::string::npos)
	{
		cout<<"representation: sphere\n";
		rightFile.close();
		rightActor=polygon3D(cloud_right,SPHERE);
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

	//remove light
	leftActor->GetProperty()->LightingOff();
	rightActor->GetProperty()->LightingOff();


	// Add the actors
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
