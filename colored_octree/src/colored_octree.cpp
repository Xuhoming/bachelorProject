#include <pcl/io/auto_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <vtkRenderWindow.h>
#include <vtkCubeSource.h>
#include <vtkGlyph3D.h>

int start_time,stop_time;

class ColoredOctreeViewer
{
public:
	ColoredOctreeViewer (std::string &filename, double resolution,bool colorMode) :// mode: true->colorless false->colored
    viz ("Colored octree visualizator"),
    cloud (new pcl::PointCloud<pcl::PointXYZRGB>()),
	clcloud(new pcl::PointCloud<pcl::PointXYZ>()),
    cloudVoxel (new pcl::PointCloud<pcl::PointXYZ>()),
    octree (resolution),
	colorless_ (colorMode)
  {
    //try to load the cloud
    if (!loadCloud(filename)) return;
    //initialize main actor
    actor = vtkSmartPointer<vtkActor>::New();
    // assign point cloud to octree
    octree.setInputCloud (clcloud);
    // add points from cloud to octree
    octree.addPointsFromInputCloud ();
    //show octree at default depth
    extractPointsAtLevel(octree.getTreeDepth());
	//render the octree cloud
	showGlyphs(std::sqrt (octree.getVoxelSquaredSideLen (octree.getTreeDepth())));
    //reset camera
    viz.resetCameraViewpoint("clcloud");

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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  //colorless cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr clcloud;
  // cloud which contains the voxel center
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVoxel;
  //octree
  pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree;
  // color mode
  bool colorless_;

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
       if(!colorless_)actor->GetProperty()->LightingOff();
      //main loop of the visualizer
      viz.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  }

  /* \brief read a pointcloud file (returns false if pbl)
   *  Also initialize the octree
   *
   */
  bool loadCloud(std::string &filename)
  {
	std::cout << "Loading file " << filename.c_str() << std::endl;
	//read cloud
	if (pcl::io::load (filename, *cloud))
	{
	  return false;
	}

	//remove NaN Points
	std::vector<int> nanIndexes;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, nanIndexes);
	std::cout << "Loaded " << cloud->points.size() << " points" << std::endl;

	clcloud->points.resize(cloud->size());
	//create the colorless cloud needed by the octree functions
	for (size_t i = 0; i < cloud->points.size (); i++)
	{
		clcloud->points[i].x=cloud->points[i].x;
		clcloud->points[i].y=cloud->points[i].y;
		clcloud->points[i].z=cloud->points[i].z;
	}

	//create octree structure
	octree.setInputCloud(clcloud);
	//update bounding box automatically
	octree.defineBoundingBox();
	//add points in the tree
	octree.addPointsFromInputCloud();
	return true;
  }

  /* \brief initialize the actor with the help of vtkGlyphs3D
   *
   */
  void showGlyphs(double voxelSideLen)
  {
	  int numberOfVoxels=cloudVoxel->points.size ();

	  //write the number of cube and their size in the viewer
	  std::string infos= boost::lexical_cast<std::string>(numberOfVoxels)+" voxels of size "+boost::lexical_cast<std::string>(floor(voxelSideLen * 1000.0f) / 1000.0f);
	  viz.addText (infos, 0, 5, 0, 0, 0, "num_size_voxel");

	  // Create points
	  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	  vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	  colors->SetName("colors");
	  colors->SetNumberOfComponents(3);

	  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	  kdtree.setInputCloud(clcloud);
	  pcl::PointXYZ searchPoint;
	  //number of nearest neighbor used to find the optimal cube color
	  int NumbNeighbor = 10;

	  double radius=voxelSideLen;
	  cout<<radius<<endl;
	  std::vector<int> nearestNeighborId;//(NumbNeighbor);
	  std::vector<float> nearestNeighborDist;//(NumbNeighbor);

	  //find the color of each voxel
	  for(pcl::PointCloud<pcl::PointXYZ>::iterator it_vox = cloudVoxel->begin();it_vox != cloudVoxel->end(); it_vox++)
	  {
		  double centerx = it_vox->x;
		  double centery = it_vox->y;
		  double centerz = it_vox->z;

		  points->InsertNextPoint(centerx,centery,centerz);

		  double numb_point,r,g,b;
		  numb_point=r=g=b=0;
		  if(colorless_)
		  {
			  r=140;
			  g=140;
			  b=140;
		  }
		  else
		  {
			  searchPoint.x=it_vox->x;
			  searchPoint.y=it_vox->y;
			  searchPoint.z=it_vox->z;

			  //if ( kdtree.nearestKSearch (searchPoint, NumbNeighbor, nearestNeighborId, nearestNeighborDist) > 0 )

			  if(kdtree.radiusSearch(searchPoint,radius,nearestNeighborId,nearestNeighborDist,0))
			  {
				  for (size_t i = 0; i < nearestNeighborId.size (); ++i){
				  r+=cloud->points[ nearestNeighborId[i] ].r;
				  g+=cloud->points[ nearestNeighborId[i] ].g;
				  b+=cloud->points[ nearestNeighborId[i] ].b;
				  }
			  }
			  r=r/nearestNeighborId.size ();
			  g=g/nearestNeighborId.size ();
			  b=b/nearestNeighborId.size ();
		  }
		  colors->InsertNextTuple3(r,g,b);
	  }

	  // Combine points and colors into a polydata
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
	  viz.setBackgroundColor(0,0,0);

	  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb (cloudVoxel, 0, 0, 0);
	  viz.addPointCloud<pcl::PointXYZ> (cloudVoxel, rgb, "Cloud");

	 // viz.addPointCloud<pcl::PointXYZRGB> (cloud);

	  viz.setShowFPS(false);
	  viz.getRenderWindow ()->Render ();

  }

  /* \brief Extracts all the points of depth = level from the octree
   *
   */
  void extractPointsAtLevel(int depth)
  {
	  cloudVoxel->points.clear();

	  pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::Iterator tree_it;
	  pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::Iterator tree_it_end = octree.end();

	  pcl::PointXYZ pt_voxel_center;

	  for (tree_it = octree.begin(depth); tree_it!=tree_it_end; ++tree_it)
	  {
		  // If the iterator is not at the right depth, continue
		  if (tree_it.getCurrentOctreeDepth () != (unsigned int) depth)
			  continue;

      // Compute the point at the center of the voxel which represents the current OctreeNode
      Eigen::Vector3f voxel_min, voxel_max;
      octree.getVoxelBounds (tree_it, voxel_min, voxel_max);

      pt_voxel_center.x = (voxel_min.x () + voxel_max.x ()) / 2.0f;
      pt_voxel_center.y = (voxel_min.y () + voxel_max.y ()) / 2.0f;
      pt_voxel_center.z = (voxel_min.z () + voxel_max.z ()) / 2.0f;

      cloudVoxel->points.push_back (pt_voxel_center);
    }
  }
};

/*brief get the min, mean and max distance between two neighbor points of the pointcloud
 *
 */
double getResolution(std::string &filename)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	double resolution;
	pcl::io::load (filename, *cloud);

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	pcl::PointXYZ searchPoint;

	std::vector<int> nearestNeighborId(2);
	std::vector<float> nearestNeighborDist(2);

	double min_neighbor=INFINITY;
	double max_neighbor=0;
	double mean_neighbor=0;

	for(pcl::PointCloud<pcl::PointXYZ>::iterator it_vox = cloud->begin();it_vox != cloud->end(); it_vox++)
	{
		searchPoint.x=it_vox->x;
		searchPoint.y=it_vox->y;
		searchPoint.z=it_vox->z;

		if ( kdtree.nearestKSearch (searchPoint, 2, nearestNeighborId, nearestNeighborDist) > 0 )
		{
			if(nearestNeighborDist[1]<min_neighbor)min_neighbor=nearestNeighborDist[1];
			else if(nearestNeighborDist[1]>max_neighbor)max_neighbor=nearestNeighborDist[1];
			mean_neighbor+=nearestNeighborDist[1];
		}
	}
	mean_neighbor/=cloud->points.size();
	printf("Mean distance value: %f will be taken as the half cube size\n",sqrt(mean_neighbor));
	resolution=2*sqrt(mean_neighbor);
	return resolution;
}
int main(int argc, char ** argv)
{
	start_time=clock();
  std::string cloud_path(argv[1]);
  float resolution=0;
  bool colorMode=false;
  switch (argc){
  	  case 2:
  		resolution=getResolution(cloud_path);
  		break;
  	  case 3:
  		if(argv[2]==std::string("cl"))
  		{
  			resolution=getResolution(cloud_path);
  			colorMode=true;
  		}
  		else
  			resolution=atof(argv[2]);
  		break;
  	  case 4:
  		  if(argv[3]==std::string("cl"))
  			resolution=atof(argv[2]);
  		colorMode=true;
  		  break;
  }
  if (!resolution)
   {
     std::cerr << "ERROR: Syntax is ./colored_octree <pcd file> [resolution] [cl]" << std::endl;
     std::cerr << "EXAMPLE for colored render: ./colored_octree andrew9-frame0000.pcd " << std::endl;
     std::cerr << "EXAMPLE for colorless render: ./colored_octree bunny.pcd 0.01 cl" << std::endl;
     return -1;
   }
  else
	  ColoredOctreeViewer v(cloud_path,resolution ,colorMode);
}
