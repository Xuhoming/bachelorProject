#include <vtkPolyData.h>
#include <vtkPLYReader.h>
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
#include <vtkInteractorStyleTrackballCamera.h>
int main ( int argc, char *argv[] )
{
clock_t tStart = clock();
  if(argc != 2 && argc!=3)
  {
    std::cout << "Usage: " << argv[0] << "  Filename(.ply) <cl>" << std::endl;
    return EXIT_FAILURE;
  }

  std::string inputFilename = argv[1];

  vtkSmartPointer<vtkPLYReader> reader =
    vtkSmartPointer<vtkPLYReader>::New();
  reader->SetFileName ( inputFilename.c_str() );

  // Visualize
  vtkSmartPointer<vtkPolyDataMapper> mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(reader->GetOutputPort());

  vtkSmartPointer<vtkActor> actor =
    vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
	if(argc==2)actor->GetProperty()->LightingOff();
  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();

vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =  vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	style->SetInteractor(renderWindowInteractor);
	renderWindowInteractor->SetInteractorStyle(style);
	style->SetCurrentRenderer(renderer);

  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderer->AddActor(actor);
  renderer->SetBackground(0,0,0); 

  renderWindow->Render();
printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);

  renderWindowInteractor->Start();
  return EXIT_SUCCESS;
}
