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
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


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
#include <iostream>
#include <fstream>
#include <ctime>
int start_time,stop_time;

class OctreeMaker
{
public:
  OctreeMaker (std::string &filename,std::string &output, double resolution,bool mode) :// mode: true->colorless false->colored

    cloud (new pcl::PointCloud<pcl::PointXYZRGB>()),
	clcloud(new pcl::PointCloud<pcl::PointXYZ>()),
    cloudVoxel (new pcl::PointCloud<pcl::PointXYZ>()),
    octree (resolution),
	colorless_ (mode)
  {
    //try to load the cloud
    if (!loadCloud(filename))
      return;

    if(mode)colorless_ =true;
    // assign point cloud to octree
    octree.setInputCloud (clcloud);

    // add points from cloud to octree
    octree.addPointsFromInputCloud ();
    //show octree at default depth
    extractPointsAtLevel(octree.getTreeDepth());

    generateOctreeCloud(output,std::sqrt (octree.getVoxelSquaredSideLen (octree.getTreeDepth())));
    stop_time=clock();
    cout << "\nExec time: " << (stop_time-start_time)/double(CLOCKS_PER_SEC)*1000<< " ms "<< endl;
    //run main loop
  }

private:
  //========================================================
  // PRIVATE ATTRIBUTES
  //========================================================

  //original cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

  //colorless cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr clcloud;

  // cloud which contains the voxel center
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVoxel;

  //octree
  pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree;

  // colorless mode
  bool colorless_;

  //========================================================


  /* \brief Helper function that read a pointcloud file (returns false if pbl)
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


void generateOctreeCloud(std::string &filename,double voxelSideLen)
{
  pcl::PointCloud<pcl::PointXYZRGB> octreeCloud;
  octreeCloud.points.resize(cloudVoxel->size());
  octreeCloud.height=1;
  octreeCloud.width=octreeCloud.size();

  int numberOfVoxels=cloudVoxel->points.size ();
  double s=voxelSideLen/2;


  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(clcloud);
  pcl::PointXYZ searchPoint;
  int NumbNeighbor = 10;

  std::vector<int> nearestNeighborId(NumbNeighbor);
  std::vector<float> nearestNeighborDist(NumbNeighbor);

  int i=0;
  for(pcl::PointCloud<pcl::PointXYZ>::iterator it_vox = cloudVoxel->begin();it_vox != cloudVoxel->end(); it_vox++)
  {

	  double centerx = it_vox->x;
	  double centery = it_vox->y;
	  double centerz = it_vox->z;

	  double numb_point,r,g,b;
	  numb_point=r=g=b=0;

	  if(colorless_)
	  {
		  r=255;
		  g=255;
		  b=255;
	  }
	  else
	  {
			searchPoint.x=it_vox->x;
			searchPoint.y=it_vox->y;
			searchPoint.z=it_vox->z;

			if ( kdtree.nearestKSearch (searchPoint, NumbNeighbor, nearestNeighborId, nearestNeighborDist) > 0 )
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


	  octreeCloud.points[i].x=centerx;
	  octreeCloud.points[i].y=centery;
	  octreeCloud.points[i].z=centerz;
	  octreeCloud.points[i].r=r;
	  octreeCloud.points[i].g=g;
	  octreeCloud.points[i].b=b;

	  i++;
  }

   pcl::io::savePCDFileASCII (filename, octreeCloud);
   fstream filestr;

    filestr.open (filename.c_str(), fstream::app);

    filestr<<"# voxel size:"+boost::lexical_cast<std::string>(voxelSideLen)<<endl;
};

  /* \brief Extracts all the points of depth = level from the octree
   *
   */
  void extractPointsAtLevel(int depth)
  {
    cloudVoxel->points.clear();

    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::Iterator tree_it;
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::Iterator tree_it_end = octree.end();

    pcl::PointXYZ pt_voxel_center;

    std::cout << "===== Extracting data at depth " << depth << "... " << std::flush;

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

double getResolution(std::string &filename)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	double resolution;
	if (pcl::io::load (filename, *cloud))
	    {
	      return false;
	    }
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

	       		  if ( kdtree.nearestKSearch (searchPoint, 2, nearestNeighborId, nearestNeighborDist) > 0 ){
	       			 if(nearestNeighborDist[1]<min_neighbor)min_neighbor=nearestNeighborDist[1];
	       			 else if(nearestNeighborDist[1]>max_neighbor)max_neighbor=nearestNeighborDist[1];
	       			 mean_neighbor+=nearestNeighborDist[1];
	       		  }
	       	  }
	       mean_neighbor/=cloud->points.size();
	       printf("Min neighbor distance: %f, max distance: %f \n",min_neighbor,max_neighbor);
	       printf("Mean distance value: %f will be taken as the half cube size\n",sqrt(mean_neighbor));
	      resolution=2*sqrt(mean_neighbor);
	return resolution;
}
int main(int argc, char ** argv)
{

  start_time=clock();
  std::string cloud_path(argv[1]);
  std::string output_path(argv[2]);
  float resolution=0;
  bool colorless=false;
  switch (argc){
  	  case 3:
  		resolution=getResolution(cloud_path);
  		break;
  	  case 4:
  		if(argv[3]==std::string("cl"))
  		{
  			resolution=getResolution(cloud_path);
  			colorless=true;
  		}
  		else
  			resolution=atof(argv[3]);
  		break;
  	  case 5:
  		  if(argv[4]==std::string("cl"))
  			resolution=atof(argv[3]);
  			colorless=true;
  		  break;
  }
  if (!resolution)
   {
     std::cerr << "ERROR: Syntax is ./pcl_octree_grid <input pcd file> <output file> [resolution] [cl]" << std::endl;
     std::cerr << "EXAMPLE for colored render: ./pcl_octree_grid andrew9-frame0000.pcd andrew_oct.pcd" << std::endl;
     std::cerr << "EXAMPLE for colorless render: ./pcl_octree_grid bunny.pcd bunny_octree.pcd 0.01 cl" << std::endl;
     return -1;
   }
  else
	  OctreeMaker v(cloud_path,output_path,resolution ,colorless) ;
}
