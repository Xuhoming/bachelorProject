//git test 3

#include <pcl/io/auto_io.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/common/shapes.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/interactor_style.h>

#include <pcl/filters/filter.h>
#include <pcl/io/boost.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkCubeSource.h>
#include <vtkCleanPolyData.h>

#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkPointData.h>
#include <vtkCubeSource.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkGlyph3D.h>
#include <vtkCellArray.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkUnsignedIntArray.h>
#include <vtkUnsignedCharArray.h>

#include <vtkDoubleArray.h>
#include <vtkCheckerboardSplatter.h>

#include <vtkXMLPolyDataWriter.h>
#include <vtkContourFilter.h>
#include <vtkGaussianSplatter.h>
#include <vtkSphereSource.h>

#include <ctime>
int start_time,stop_time;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::visualization::PCLVisualizer viz;

void run()
 {
   while (!viz.wasStopped())
   {

     //main loop of the visualizer
     viz.spinOnce(100);
     boost::this_thread::sleep(boost::posix_time::microseconds(100000));
   }
 }
void surface_recon(std::string type)
{
	  // Create points
	  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

	  for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin();it != cloud->end(); it++)
	  {
	     points->InsertNextPoint(it->x,it->y,it->z);

	  }

	  vtkSmartPointer<vtkPolyData> polydata =  vtkSmartPointer<vtkPolyData>::New();
	  polydata->SetPoints(points);

	  vtkSmartPointer<vtkContourFilter> surface =   vtkSmartPointer<vtkContourFilter>::New();

	  if(type==std::string("gauss"))
	  {
		  vtkSmartPointer<vtkGaussianSplatter> splatter =  vtkSmartPointer<vtkGaussianSplatter>::New();

		  splatter->SetInputData(polydata);

		  unsigned int n = 150;
		  splatter->SetSampleDimensions(n,n,n);
		  splatter->SetRadius(.02);
		  splatter->SetExponentFactor(-10);
		  splatter->SetEccentricity(2);
		  splatter->Update();

		  surface->SetInputConnection(splatter->GetOutputPort());
		  surface->SetValue(0,.95* splatter->GetRadius() );

	  }

	  else if(type==std::string("check"))
	  {
		  vtkSmartPointer<vtkCheckerboardSplatter> splatter =  vtkSmartPointer<vtkCheckerboardSplatter>::New();

		  splatter->SetInputData(polydata);

		  unsigned int n = 150;
		  splatter->SetSampleDimensions(n,n,n);
		  splatter->SetRadius(.02);
		  splatter->SetExponentFactor(-10);
		  splatter->SetEccentricity(2);
		  splatter->Update();

		  surface->SetInputConnection(splatter->GetOutputPort());
		  surface->SetValue(0, 0.01);
	  }

	  // Create a mapper and actor
	  vtkSmartPointer<vtkPolyDataMapper> mapper =   vtkSmartPointer<vtkPolyDataMapper>::New();
	  mapper->SetInputConnection(surface->GetOutputPort());

	  vtkSmartPointer<vtkActor> actor =  vtkSmartPointer<vtkActor>::New();
	  actor->SetMapper(mapper);

	  viz.getRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->AddActor(actor);

	  viz.setShowFPS(false);

	  viz.getRenderWindow ()->Render ();
}

int main(int argc, char ** argv)
{
	start_time=clock();
	if(argc!=3)
	{
		printf("./surface_recon <gauss/check> cloudpath \n");
		return EXIT_FAILURE;
	}

	std::string cloud_path(argv[2]);
	std::string recon_type(argv[1]);
	printf("loading cloud %s \n",cloud_path.c_str());
	pcl::io::load (cloud_path, *cloud);
	viz.resetCameraViewpoint("cloud");

	viz.addPointCloud (cloud, "original_cloud");

	surface_recon(recon_type);

	stop_time=clock();
	cout << "\nExec time: " << (stop_time-start_time)/double(CLOCKS_PER_SEC)*1000<< " ms "<< endl;
	run();


	return EXIT_SUCCESS;

}
