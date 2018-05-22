#include <vtkPolyData.h>
#include <vtkDataSetReader.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <vtkCamera.h>
#include <vtkInteractorStyleTrackballCamera.h>
int main ( int argc, char *argv[] )
{
clock_t tStart = clock();
  if(argc != 2 && argc!=3)
  {
    std::cout << "Usage: " << argv[0] << "  Filename(.ply) <cl>" << std::endl;
    return EXIT_FAILURE;
  }
vtkSmartPointer<vtkCamera> sharedCamera = vtkSmartPointer<vtkCamera>::New();

  std::string inputFilename1 = argv[1];

  vtkSmartPointer<vtkDataSetReader> reader1 = vtkSmartPointer<vtkDataSetReader>::New();
  reader1->SetFileName ( inputFilename1.c_str() );

  // Visualize
  vtkSmartPointer<vtkPolyDataMapper> mapper1 = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper1->SetInputConnection(reader1->GetOutputPort());

  vtkSmartPointer<vtkActor> actor1 = vtkSmartPointer<vtkActor>::New();
  actor1->GetProperty()->LightingOff();
  actor1->SetMapper(mapper1);
  vtkSmartPointer<vtkRenderer> renderer1 = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer1);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =vtkSmartPointer<vtkRenderWindowInteractor>::New();

vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =  vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	style->SetInteractor(renderWindowInteractor);
	renderWindowInteractor->SetInteractorStyle(style);
	style->SetCurrentRenderer(renderer1);

  renderWindowInteractor->SetRenderWindow(renderWindow);
renderer1->SetActiveCamera(sharedCamera);
  renderer1->AddActor(actor1);
renderer1->SetViewport(0,0,0.5,1);

std::string inputFilename2 = argv[2];

  vtkSmartPointer<vtkDataSetReader> reader2 = vtkSmartPointer<vtkDataSetReader>::New();
  reader2->SetFileName ( inputFilename2.c_str() );

  // Visualize
  vtkSmartPointer<vtkPolyDataMapper> mapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper2->SetInputConnection(reader2->GetOutputPort());

  vtkSmartPointer<vtkActor> actor2 = vtkSmartPointer<vtkActor>::New();
  actor2->GetProperty()->LightingOff();
  actor2->SetMapper(mapper2);
  vtkSmartPointer<vtkRenderer> renderer2 = vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(renderer2);
renderer2->SetActiveCamera(sharedCamera);
renderer2->SetViewport(0.5,0,1,1);


  renderer2->AddActor(actor2);


  renderWindow->Render();
printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
std::ofstream log("logfile.txt", std::ios_base::app | std::ios_base::out);

  renderWindowInteractor->Start();
  return EXIT_SUCCESS;
}
