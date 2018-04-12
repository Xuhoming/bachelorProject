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
#include <vtkScalarsToColors.h>

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

#include <string.h>
#include <list>
#include <ctime>
int start_time,stop_time;

class VoxelViewer
{
public:
  VoxelViewer (std::string &filename,bool colorless) :
    viz ("Colored octree visualizator"),
	cloud (new pcl::PointCloud<pcl::PointXYZRGB>()),
	clcloud (new pcl::PointCloud<pcl::PointXYZ>()),
	_colorless(colorless)
  {

    //try to load the cloud
    if (!loadCloud(filename))
      return;

    actor = vtkSmartPointer<vtkActor>::New();
    showGlyphs();

    //reset camera
    viz.resetCameraViewpoint("cloud");
    stop_time=clock();
    cout << "\nExec time: " << (stop_time-start_time)/double(CLOCKS_PER_SEC)*1000<< " ms "<< endl;
    //run main loop
    run();
  }

private:
  //========================================================
  // PRIVATE ATTRIBUTES
  //========================================================
  //visualizer

  pcl::visualization::PCLVisualizer viz;
  //original cloud

  // cloud which contains the voxel center
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

  pcl::PointCloud<pcl::PointXYZ>::Ptr clcloud;

  bool _colorless;

  //viewer actor
  vtkSmartPointer<vtkActor> actor ;
  //========================================================

  /* \brief Graphic loop for the viewer
   *
   */
  void run()
  {
    while (!viz.wasStopped())
    {
      if(!_colorless)actor->GetProperty()->LightingOff();
      //main loop of the visualizer
      viz.spinOnce(100);

      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  };

  /* \brief Helper function that read a pointcloud file (returns false if pbl)
   *  Also initialize the octree
   *
   */
  bool loadCloud(std::string &filename)
  {
    std::cout << "Loading file " << filename.c_str() << std::endl;
    //read cloud
    pcl::io::load (filename, *cloud);

    clcloud->points.resize(cloud->size());
       //create the colorless cloud needed by the octree functions
       for (size_t i = 0; i < cloud->points.size (); i++)
           {
   			clcloud->points[i].x=cloud->points[i].x;
   			clcloud->points[i].y=cloud->points[i].y;
   			clcloud->points[i].z=cloud->points[i].z;
           }

    std::cout << "Loaded " << cloud->points.size() << " points" << std::endl;
    return true;
  }


	void showGlyphs()
	{
	  double mean_dist,max_dist=0;;
	  int id=0;
	  // Create points
	  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

	  // setup scales
	  vtkSmartPointer<vtkFloatArray> scales = vtkSmartPointer<vtkFloatArray>::New();
	  scales->SetName("scales");

	  vtkSmartPointer<vtkFloatArray> col = vtkSmartPointer<vtkFloatArray>::New();
	  col->SetName("col");

	  vtkSmartPointer<vtkLookupTable> lut = vtkSmartPointer<vtkLookupTable>::New();
	  lut->SetNumberOfTableValues(cloud->points.size());
	  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	  kdtree.setInputCloud(clcloud);
	  pcl::PointXYZ searchPoint;
	  int NumbNeighbor = 5;

	  std::vector<int> nearestNeighborId(NumbNeighbor);
	  std::vector<float> nearestNeighborDist(NumbNeighbor);



	  for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it_vox = cloud->begin();it_vox != cloud->end(); it_vox++)
	  {
		  mean_dist=0;
		  max_dist=0;
		  double r,g,b;
		  double centerx = it_vox->x;
		  double centery = it_vox->y;
		  double centerz = it_vox->z;

		  points->InsertNextPoint(centerx,centery,centerz);

		  searchPoint.x=it_vox->x;
		  searchPoint.y=it_vox->y;
		  searchPoint.z=it_vox->z;

		  if ( kdtree.nearestKSearch (searchPoint, NumbNeighbor, nearestNeighborId, nearestNeighborDist) > 0 )
		  {
			  for (size_t i = 0; i < nearestNeighborId.size (); ++i)
			  {
				  mean_dist+=sqrt(nearestNeighborDist[i]);
				  if(sqrt(nearestNeighborDist[i])>max_dist)max_dist=sqrt(nearestNeighborDist[i]);
			  }
		  }
		  mean_dist/=NumbNeighbor-1;
		  if(_colorless)
		  {
			  r=g=b=1;

		  }
		  else
		  {
			   r =it_vox->r/255.0;
			   g =it_vox->g/255.0;
			   b =it_vox->b/255.0;
		  }
		  col->InsertNextValue(id);
		  lut->SetTableValue(id,r,g,b,1);
		  scales->InsertNextValue(mean_dist);
		  id++;
	  }

		   // grid structured to append center, radius and color label
		      vtkSmartPointer<vtkUnstructuredGrid> grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
		      grid->SetPoints(points);
		      grid->GetPointData()->AddArray(scales);
		      grid->GetPointData()->SetActiveScalars("scales"); // !!!to set radius first
		      grid->GetPointData()->AddArray(col);

		   vtkSmartPointer<vtkCubeSource> cubeSource =  vtkSmartPointer<vtkCubeSource>::New();

		   vtkSmartPointer<vtkGlyph3D> glyph3D =  vtkSmartPointer<vtkGlyph3D>::New();
		   glyph3D->SetInputData(grid);
		   glyph3D->SetSourceConnection(cubeSource->GetOutputPort());

		   // Create a mapper
		   vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		   mapper->SetInputConnection(glyph3D->GetOutputPort());

		   mapper->SetScalarModeToUsePointFieldData();
		   mapper->SetScalarRange(0,cloud->points.size());
		   mapper->SelectColorArray("col");
		   mapper->SetLookupTable(lut);

		   actor->SetMapper(mapper);

		   viz.getRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->AddActor(actor);

		   viz.setShowFPS(false);
		   // Render and interact
		   viz.getRenderWindow ()->Render ();
	}
};


int main(int argc, char ** argv)
{
	start_time=clock();
	std::string cloud_path(argv[1]);

	if(argc==2)VoxelViewer v(cloud_path,false);
	else if(argc==3 && argv[2]==std::string("cl"))VoxelViewer v(cloud_path,true);
	else
   {
     std::cerr << "ERROR: Syntax is ./colored_octree <pcd file> [cl]" << std::endl;
     std::cerr << "EXAMPLE for colored render: ./colored_octree andrew9_octreed.pcd " << std::endl;
     std::cerr << "EXAMPLE for colorless render: ./colored_octree bunny_octreed.pcd cl" << std::endl;
     return -1;
   }

}
