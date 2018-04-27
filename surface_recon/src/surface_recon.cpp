#include <pcl/io/auto_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d_omp.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkTensorGlyph.h>
#include <vtkOpenGLPolyDataMapper.h>
#include <vtkDoubleArray.h>
#include <vtkContourFilter.h>
#include <vtkGaussianSplatter.h>
#include <vtkRegularPolygonSource.h>
#include <vtkReverseSense.h>
#include <vtkSurfaceReconstructionFilter.h>
#include <vtkDelaunay3D.h>


//#include <pcl/visualization/interactor_style.h>
//#include <pcl/point_types.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/point_types.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/filters/filter.h>
//#include <pcl/io/boost.h>
//#include <pcl/io/vtk_io.h>
//#include <vtkVersion.h>
//#include <vtkSmartPointer.h>
//#include <vtkPointData.h>
//#include <vtkPolyData.h>
//#include <vtkPoints.h>
//#include <vtkGlyph3D.h>
//#include <vtkCellArray.h>
//#include <vtkPolyDataMapper.h>
//#include <vtkActor.h>
//#include <vtkRenderer.h>
//#include <vtkUnsignedIntArray.h>
//#include <vtkUnsignedCharArray.h>
//#include <vtkXMLPolyDataWriter.h>
//#include <vtkSphereSource.h>
//#include <ctime>
//#include <cmath>

int start_time,stop_time;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb (new pcl::PointCloud<pcl::PointXYZRGB>);

bool colorless=false;
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

void delaunay()
{
	// Create points
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

	for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin();it != cloud->end(); it++)
	{
		points->InsertNextPoint(it->x,it->y,it->z);
	}

	vtkSmartPointer<vtkPolyData> polydata =  vtkSmartPointer<vtkPolyData>::New();
	polydata->SetPoints(points);

	// Generate a tetrahedral mesh from the input points. By
	// default, the generated volume is the convex hull of the points.
	vtkSmartPointer<vtkDelaunay3D> delaunay3D =
	vtkSmartPointer<vtkDelaunay3D>::New();
	delaunay3D->SetInputData(polydata);
	vtkSmartPointer<vtkDataSetMapper> delaunayMapper =
	vtkSmartPointer<vtkDataSetMapper>::New();
	delaunayMapper->SetInputConnection(delaunay3D->GetOutputPort());

	vtkSmartPointer<vtkActor> delaunayActor =
	vtkSmartPointer<vtkActor>::New();
	delaunayActor->SetMapper(delaunayMapper);
	delaunayActor->GetProperty()->SetColor(1,0,0);


	viz.getRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->AddActor(delaunayActor);

	viz.setShowFPS(false);

	viz.getRenderWindow ()->Render ();
}

void fast_tri()
{
	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (0.025);

	// Set typical values for the parameters
	gp3.setMu (3);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	viz.addPolygonMesh(triangles);
}
void gauss()
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


	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper =   vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(surface->GetOutputPort());

	vtkSmartPointer<vtkActor> actor =  vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	viz.getRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->AddActor(actor);

	viz.setShowFPS(false);

	viz.getRenderWindow ()->Render ();
}

void surf_rec_filter()
{
	// Create points
		vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

		for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin();it != cloud->end(); it++)
		{
			points->InsertNextPoint(it->x,it->y,it->z);
		}

		vtkSmartPointer<vtkPolyData> polydata =  vtkSmartPointer<vtkPolyData>::New();
		polydata->SetPoints(points);

		vtkSmartPointer<vtkSurfaceReconstructionFilter> surface_rec = vtkSmartPointer<vtkSurfaceReconstructionFilter>::New();

		vtkSmartPointer<vtkContourFilter> cont_filter =   vtkSmartPointer<vtkContourFilter>::New();


		surface_rec->SetInputData(polydata);


		cont_filter->SetInputConnection(surface_rec->GetOutputPort());
		cont_filter->SetValue(0,0.0);

		vtkSmartPointer<vtkReverseSense> reverse =   vtkSmartPointer<vtkReverseSense>::New();

		reverse->SetInputConnection(cont_filter->GetOutputPort());
		reverse->ReverseCellsOn();
		reverse->ReverseNormalsOn();


		// Create a mapper and actor
		vtkSmartPointer<vtkPolyDataMapper> mapper =   vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputConnection(reverse->GetOutputPort());
		mapper->SetScalarVisibility(0);



		vtkSmartPointer<vtkActor> actor =vtkSmartPointer<vtkActor>::New();
		actor->SetMapper(mapper);
		actor->GetProperty()->SetColor(1,0,0);


		viz.getRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->AddActor(actor);

		viz.setShowFPS(false);

		viz.getRenderWindow ()->Render ();

}



void point_based(double density,int numbSide)
{
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	ne.setRadiusSearch (10*density);

	// Compute the features
	ne.compute (*cloud_normals);

	vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();
	normals->SetName("normals");
	normals->SetNumberOfComponents(3);

	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetName("colors");
	colors->SetNumberOfComponents(3);

	vtkSmartPointer<vtkDoubleArray> tensors = vtkSmartPointer<vtkDoubleArray>::New();
	tensors->SetNumberOfTuples(3);
	tensors->SetNumberOfComponents(9);


	double normal[3];
	double rotAxis[3];
	double theta;

	pcl::PointXYZ searchPoint;
	int NumbNeighbor = 5;

	std::vector<int> nearestNeighborId(NumbNeighbor);
	std::vector<float> nearestNeighborDist(NumbNeighbor);
	double max_dist=0;
	double scale;
	for(int i=0;i<cloud->points.size();i++)
	{
		max_dist=0;
		points->InsertNextPoint(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);
		if(!colorless)colors->InsertNextTuple3(cloudrgb->points[i].r,cloudrgb->points[i].g,cloudrgb->points[i].b);
		normal[0]=cloud_normals->points[i].normal_x;
		normal[1]=cloud_normals->points[i].normal_y;
		normal[2]=cloud_normals->points[i].normal_z;

		//normal cross e_z product
		rotAxis[0] = normal[1] ;
		rotAxis[1] = -normal[0] ;
		rotAxis[2] = 0;
		double norm=sqrt(rotAxis[0]*rotAxis[0]+rotAxis[1]*rotAxis[1]+rotAxis[2]*rotAxis[2]);
		rotAxis[0] =rotAxis[0]/norm;
		rotAxis[1] =rotAxis[1]/norm;
		rotAxis[2] =rotAxis[2]/norm;

		theta=std::acos(normal[2]);

		searchPoint.x=cloud->points[i].x;
		searchPoint.y=cloud->points[i].y;
		searchPoint.z=cloud->points[i].z;
		if ( tree->nearestKSearch (searchPoint, NumbNeighbor, nearestNeighborId, nearestNeighborDist) > 0 )
		{
			  for (size_t i = 0; i < nearestNeighborId.size (); ++i)
			  {

				  if(nearestNeighborDist[i]>max_dist)max_dist=nearestNeighborDist[i];
			  }
			  max_dist=sqrt(max_dist);
		}

		scale=max_dist<density?density: max_dist;

		if(!normal[0]&&!normal[1]&& abs(normal[2])==1)
			tensors->InsertNextTuple9(scale,0,0,0,scale,0,0,0,scale);
		else
		tensors->InsertNextTuple9((cos(theta)+rotAxis[0]*rotAxis[0]*(1-cos(theta)))*scale,(rotAxis[0]*rotAxis[1]*(1-cos(theta))-rotAxis[2]*sin(theta))*scale,(rotAxis[0]*rotAxis[2]*(1-cos(theta))+rotAxis[1]*sin(theta))*scale,
								  (rotAxis[1]*rotAxis[0]*(1-cos(theta))+rotAxis[2]*sin(theta))*scale,(cos(theta)+rotAxis[1]*rotAxis[1]*(1-cos(theta)))*scale,(rotAxis[1]*rotAxis[2]*(1-cos(theta))-rotAxis[0]*sin(theta))*scale,
								  (rotAxis[2]*rotAxis[0]*(1-cos(theta))-rotAxis[1]*sin(theta))*scale,(rotAxis[2]*rotAxis[1]*(1-cos(theta))+rotAxis[0]*sin(theta))*scale,(cos(theta)+rotAxis[2]*rotAxis[2]*(1-cos(theta)))*scale);
	}

	vtkSmartPointer<vtkPolyData> polydata =  vtkSmartPointer<vtkPolyData>::New();
	polydata->SetPoints(points);
	polydata->GetPointData()->SetTensors(tensors);
	if(!colorless) polydata->GetPointData()->SetScalars(colors);

	// Create a circle
	vtkSmartPointer<vtkRegularPolygonSource> polygonSource =  vtkSmartPointer<vtkRegularPolygonSource>::New();

	polygonSource->SetNumberOfSides(numbSide);
	polygonSource->SetRadius(1);
	polygonSource->GeneratePolylineOff();


	polygonSource->Update();

	vtkSmartPointer<vtkTensorGlyph> tensorGlyph = vtkSmartPointer<vtkTensorGlyph>::New();
	tensorGlyph->SetInputData(polydata);
	tensorGlyph->SetSourceConnection(polygonSource->GetOutputPort());
	tensorGlyph->ColorGlyphsOff();
	tensorGlyph->ThreeGlyphsOff();
	tensorGlyph->ExtractEigenvaluesOff();
	tensorGlyph->SetColorModeToScalars();
	tensorGlyph->SymmetricOff();
	tensorGlyph->Update();


	// Visualize

	vtkSmartPointer<vtkOpenGLPolyDataMapper> mapper =  vtkSmartPointer<vtkOpenGLPolyDataMapper>::New();
	mapper->SetInputData(tensorGlyph->GetOutput());

	vtkSmartPointer<vtkActor> actor =  vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	actor->GetProperty()->SetColor(.8,.8,.8);
	viz.getRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->AddActor(actor);


	viz.getRenderWindow ()->Render ();
	viz.setShowFPS(false);
	//	viz.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, cloud_normals, 1, 0.01, "normals1", 0);

//		viz.addCoordinateSystem(0.5);

//		const char *info = viz.getRenderWindow()->ReportCapabilities();
//		cerr << info;


}
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
	       printf("Mean distance value: %f will be taken as the radius\n",sqrt(mean_neighbor));
	      resolution=sqrt(mean_neighbor);
	return resolution;
}

int main(int argc, char ** argv)
{
	start_time=clock();
	if(argc!=3 && argc != 4 && argc!= 5)
	{
		printf("./surface_recon cloudpath  <fast/point/gauss/delaunay/surf_filter>  [number of sides] [cl] \n");
		printf("exemple: ./surface_recon bunny.pcd point 20 cl\n");
		return EXIT_FAILURE;
	}
	std::string cloud_path(argv[1]);
	std::string recon_type(argv[2]);
	pcl::io::load (cloud_path, *cloud);

	if(argv[3]==std::string("cl") ||(argc==5&&argv[4]==std::string("cl") ))
		colorless=true;

	if(!colorless)
		pcl::io::load (cloud_path, *cloudrgb);

	printf("loading cloud %s \n",cloud_path.c_str());

	viz.resetCameraViewpoint("cloud");

//	viz.addPointCloud (cloud, "original_cloud");

	if(recon_type==std::string("delaunay"))delaunay();
	if(recon_type==std::string("fast"))fast_tri();
	if(recon_type==std::string("gauss"))gauss();
	if(recon_type==std::string("surf_filter"))surf_rec_filter();
	if(recon_type==std::string("point"))
	{
		point_based(getResolution(cloud_path),atoi(argv[3]));
	}

	stop_time=clock();
	cout << "\nExec time: " << (stop_time-start_time)/double(CLOCKS_PER_SEC)*1000<< " ms "<< endl;
	run();

	return EXIT_SUCCESS;
}
