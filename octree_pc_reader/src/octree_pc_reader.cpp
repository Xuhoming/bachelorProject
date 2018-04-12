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

#include <string.h>
#include <list>
#include <ctime>
int start_time,stop_time;

class OctreeViewer
{
public:
  OctreeViewer (std::string &filename, double resolution,bool colorless) :
    viz ("Colored octree visualizator"),
    cloudVoxel (new pcl::PointCloud<pcl::PointXYZRGB>()),
	_colorless(colorless)
  {

    //try to load the cloud
    if (!loadCloud(filename))
      return;

    actor = vtkSmartPointer<vtkActor>::New();
    showGlyphs(resolution);

    //reset camera
    viz.resetCameraViewpoint("cloudVoxel");
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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudVoxel;

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
    pcl::io::load (filename, *cloudVoxel);


    std::cout << "Loaded " << cloudVoxel->points.size() << " points" << std::endl;
    return true;
  }


	void showGlyphs(double voxelSideLen)
	{
	  int numberOfVoxels=cloudVoxel->points.size ();
	  double s=voxelSideLen/2;
	  std::string infos= boost::lexical_cast<std::string>(numberOfVoxels)+" voxels of size "+boost::lexical_cast<std::string>(floor(voxelSideLen * 1000.0f) / 1000.0f);
	  viz.addText (infos, 0, 5, 1.0, 1.0, 1.0, "num_size_voxel");

	  // Create points
	  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	  vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	  colors->SetName("colors");
	  colors->SetNumberOfComponents(3);

	  for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it_vox = cloudVoxel->begin();it_vox != cloudVoxel->end(); it_vox++)
	  {
		  double centerx = it_vox->x;
		  double centery = it_vox->y;
		  double centerz = it_vox->z;

		  points->InsertNextPoint(centerx,centery,centerz);

		  double r =it_vox->r;
		  double g =it_vox->g;
		  double b =it_vox->b;

		  colors->InsertNextTuple3(r,g,b);
	  }
	   // Combine into a polydata
	   vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	   polydata->SetPoints(points);
	   polydata->GetPointData()->SetScalars(colors);

	   vtkSmartPointer<vtkCubeSource> cubeSource =  vtkSmartPointer<vtkCubeSource>::New();
	   cubeSource->SetXLength(voxelSideLen);
	   cubeSource->SetYLength(voxelSideLen);
	   cubeSource->SetZLength(voxelSideLen);

	   vtkSmartPointer<vtkGlyph3D> glyph3D =  vtkSmartPointer<vtkGlyph3D>::New();
	   glyph3D->SetColorModeToColorByScalar();
	   glyph3D->SetSourceConnection(cubeSource->GetOutputPort());

	   glyph3D->SetInputData(polydata);

	   glyph3D->ScalingOff();
	   glyph3D->Update();

	   // Create a mapper
	   vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	   mapper->SetInputConnection(glyph3D->GetOutputPort());

	   actor->SetMapper(mapper);
	   actor->GetProperty()->BackfaceCullingOn();

	   viz.getRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->AddActor(actor);

	   viz.setShowFPS(false);
	   // Render and interact
	   viz.getRenderWindow ()->Render ();
	}
};


double extract_voxel_size(std::string &cloud_path)
{
	double voxel_size;

	 std::ifstream read(cloud_path.c_str(), std::ios_base::ate );//open file
		    std::string tmp;
		    int length = 0;

		    char c = '\0';

		    if( read )
		    {
		        length = read.tellg();//Get file size

		        // loop backward over the file

		        for(int i = length-2; i > 0; i-- )
		        {
		            read.seekg(i);
		            c = read.get();
		            if( c == '\r' || c == '\n' )//new line?
		                 break;
		        }

		        std::getline(read, tmp);//read last line
		        std::string s = "# voxel size:";

		        std::string::size_type i = tmp.find(s);

		        if (i != std::string::npos)tmp.erase(i, s.length());

		        std::cout <<"voxel size extracted: "<< tmp << std::endl; // print it

		    }
		    voxel_size=atof(tmp.c_str());
	return voxel_size;

};

int main(int argc, char ** argv)
{
	printf("%f\n",INFINITY);
	start_time=clock();
	std::string cloud_path(argv[1]);

	if(argc==2)OctreeViewer v(cloud_path,extract_voxel_size(cloud_path),false);
	else if(argc==3 && argv[2]==std::string("cl"))OctreeViewer v(cloud_path,extract_voxel_size(cloud_path),true);
	else
   {
     std::cerr << "ERROR: Syntax is ./colored_octree <pcd file> [cl]" << std::endl;
     std::cerr << "EXAMPLE for colored render: ./colored_octree andrew9_octreed.pcd " << std::endl;
     std::cerr << "EXAMPLE for colorless render: ./colored_octree bunny_octreed.pcd cl" << std::endl;
     return -1;
   }

}
