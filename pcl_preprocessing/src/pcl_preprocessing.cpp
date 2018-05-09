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
#include <iostream>
#include <fstream>
#include <limits>
#include <vtkCubeSource.h>
#include <vtkGlyph3D.h>

#include <vtkPLYWriter.h>
#include <vtkPLYReader.h>

int start_time,stop_time;

enum type{SQUARE,DISK,CUBE,SPHERE};
bool colorless=false;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB (new pcl::PointCloud<pcl::PointXYZRGB>);

void regularpolygon2D(double density,int type,std::string &savefile)
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
	double scale,norm;
	double rotation[9];
	for(int i=0;i<cloud->points.size();i++)
	{
		max_dist=0;
		points->InsertNextPoint(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);
		normal[0]=cloud_normals->points[i].normal_x;
		normal[1]=cloud_normals->points[i].normal_y;
		normal[2]=cloud_normals->points[i].normal_z;

		//normal cross e_z product
		rotAxis[0] = normal[1] ;
		rotAxis[1] = -normal[0] ;
		rotAxis[2] = 0;
		norm=sqrt(rotAxis[0]*rotAxis[0]+rotAxis[1]*rotAxis[1]+rotAxis[2]*rotAxis[2]);
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

		if(!normal[0]&&!normal[1]&& abs(normal[2])==1){
			rotation[0]=scale;
			rotation[1]=0;
			rotation[2]=0;
			rotation[3]=0;
			rotation[4]=scale;
			rotation[5]=0;
			rotation[6]=0;
			rotation[7]=0;
			rotation[8]=scale;
		}
		else
		{
			rotation[0]=(cos(theta)+rotAxis[0]*rotAxis[0]*(1-cos(theta)))*scale;
			rotation[1]=(rotAxis[0]*rotAxis[1]*(1-cos(theta))-rotAxis[2]*sin(theta))*scale;
			rotation[2]=(rotAxis[0]*rotAxis[2]*(1-cos(theta))+rotAxis[1]*sin(theta))*scale;
			rotation[3]=(rotAxis[1]*rotAxis[0]*(1-cos(theta))+rotAxis[2]*sin(theta))*scale;
			rotation[4]=(cos(theta)+rotAxis[1]*rotAxis[1]*(1-cos(theta)))*scale;
			rotation[5]=(rotAxis[1]*rotAxis[2]*(1-cos(theta))-rotAxis[0]*sin(theta))*scale;
			rotation[6]=(rotAxis[2]*rotAxis[0]*(1-cos(theta))-rotAxis[1]*sin(theta))*scale;
			rotation[7]=(rotAxis[2]*rotAxis[1]*(1-cos(theta))+rotAxis[0]*sin(theta))*scale;
			rotation[8]=(cos(theta)+rotAxis[2]*rotAxis[2]*(1-cos(theta)))*scale;
		}
		tensors->InsertNextTuple9(rotation[0],rotation[1],rotation[2],rotation[3],rotation[4],rotation[5],rotation[6],rotation[7],rotation[8]);
	}

	vtkSmartPointer<vtkUnstructuredGrid> grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
	grid->SetPoints(points);
	grid->GetPointData()->SetTensors(tensors);

	// Create a polygon
	vtkSmartPointer<vtkRegularPolygonSource> polygonSource =  vtkSmartPointer<vtkRegularPolygonSource>::New();
	if(type==SQUARE)
		polygonSource->SetNumberOfSides(4);
	if(type==DISK)
		polygonSource->SetNumberOfSides(15);
	polygonSource->SetRadius(1);
	polygonSource->GeneratePolylineOff();
	polygonSource->Update();

	vtkSmartPointer<vtkTensorGlyph> tensorGlyph = vtkSmartPointer<vtkTensorGlyph>::New();
	tensorGlyph->SetInputData(grid);
	tensorGlyph->SetSourceConnection(polygonSource->GetOutputPort());
	tensorGlyph->ColorGlyphsOff();
	tensorGlyph->ThreeGlyphsOff();
	tensorGlyph->ExtractEigenvaluesOff();
	tensorGlyph->SymmetricOff();
	tensorGlyph->Update();

	vtkCellArray *polys;
	 vtkPoints *inPts;
	 vtkPolyData *input = tensorGlyph->GetOutput();

	 polys = input->GetPolys();
	 inPts = input->GetPoints();
	 double dpoint[3];

	 ofstream writtingFile (savefile.c_str());
	 writtingFile <<"ply \n";
	 writtingFile <<"format ascii 1.0 \n";
	 writtingFile <<"obj_info vtkPolyData points and polygons: vtk4.0 \n";
	 writtingFile <<"element vertex "<<inPts->GetNumberOfPoints() <<"\n";
	 writtingFile <<"property float x \n";
	 writtingFile <<"property float y \n";
	 writtingFile <<"property float z \n";
	 writtingFile <<"element face "<< polys->GetNumberOfCells() <<"\n";
	 writtingFile <<"property list uchar int vertex_indices \n";
	 if(!colorless)
	 {
		 writtingFile <<"property uchar red \n";
		 writtingFile <<"property uchar green \n";
		 writtingFile <<"property uchar blue \n";
	 }

	 writtingFile <<"end_header \n";

	 for(int i=0;i<inPts->GetNumberOfPoints();i++)
	 {
		 inPts->GetPoint(i,dpoint);
		 writtingFile <<dpoint[0]<<" "<<dpoint[1]<<" "<<dpoint[2]<<"\n";
	 }
	 int id;
	 vtkIdType npts = 0;
	 vtkIdType *pts = 0;
	 double rgb[3];
	 for (polys->InitTraversal(), id = 0; id < polys->GetNumberOfCells(); id++)
	 {
		 polys->GetNextCell(npts,pts);
		 writtingFile <<npts<<" ";
		 for (int j=0; j<npts; j++)
		  {
			 writtingFile <<(int)pts[j]<< " ";
		  }
		 if(!colorless)writtingFile <<static_cast<int>(cloudRGB->points[id].r)<<" "<<static_cast<int>(cloudRGB->points[id].g)<<" "<<static_cast<int>(cloudRGB->points[id].b)<<" "<<"\n";
	 	 else writtingFile <<"\n";

	 }


//	// Visualize
//
//	vtkSmartPointer<vtkOpenGLPolyDataMapper> mapper =  vtkSmartPointer<vtkOpenGLPolyDataMapper>::New();
//	mapper->SetInputData(tensorGlyph->GetOutput());
//	mapper->SetInputConnection(tensorGlyph->GetOutputPort());
//	mapper->SetScalarModeToUsePointFieldData();
//
//
//	vtkSmartPointer<vtkActor> actor =  vtkSmartPointer<vtkActor>::New();
//	actor->SetMapper(mapper);
//
//
//	  vtkSmartPointer<vtkRenderer> renderer =
//	    vtkSmartPointer<vtkRenderer>::New();
//	  renderer->AddActor(actor);
//	  renderer->SetBackground(0.1, 0.2, 0.4);
//	  // Zoom in a little by accessing the camera and invoking its "Zoom" method.
//	  renderer->ResetCamera();
//
//
//	  // The render window is the actual GUI window
//	  // that appears on the computer screen
//	  vtkSmartPointer<vtkRenderWindow> renderWindow =
//	    vtkSmartPointer<vtkRenderWindow>::New();
//	  renderWindow->SetSize(200, 200);
//	  renderWindow->AddRenderer(renderer);
//
//	  // The render window interactor captures mouse events
//	  // and will perform appropriate camera or actor manipulation
//	  // depending on the nature of the events.
//	  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
//	    vtkSmartPointer<vtkRenderWindowInteractor>::New();
//	  renderWindowInteractor->SetRenderWindow(renderWindow);
//
//	  // This starts the event loop and as a side effect causes an initial render.
//	  renderWindowInteractor->Start();

}

void regularpolygon3D(int type,std::string &savefile)
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
	lut->SetRange(0,cloud->points.size());

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	pcl::PointXYZ searchPoint;
	int NumbNeighbor = 5;

	std::vector<int> nearestNeighborId(NumbNeighbor);
	std::vector<float> nearestNeighborDist(NumbNeighbor);

	for(int i=0;i<cloud->points.size();i++)
	{
	  mean_dist=0;
	  max_dist=0;
	  double r,g,b;


	  points->InsertNextPoint(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);
	  searchPoint.x=cloud->points[i].x;
	  searchPoint.y=cloud->points[i].y;
	  searchPoint.z=cloud->points[i].z;

	  if ( kdtree.nearestKSearch (searchPoint, NumbNeighbor, nearestNeighborId, nearestNeighborDist) > 0 )
	  {
		  for (size_t j = 0; j < nearestNeighborId.size (); ++j)
		  {
			  mean_dist+=sqrt(nearestNeighborDist[j]);
			  if(sqrt(nearestNeighborDist[j])>max_dist)max_dist=sqrt(nearestNeighborDist[j]);
		  }
	  }
	  mean_dist/=NumbNeighbor-1;
	  if(colorless)
	  {
		  r=g=b=1;
	  }
	  else
	  {
		  r =cloudRGB->points[i].r/255.0;
		  g =cloudRGB->points[i].g/255.0;
		  b =cloudRGB->points[i].b/255.0;
	  }
	  col->InsertNextValue(id);
	  lut->SetTableValue(id,r,g,b);
	  scales->InsertNextValue(1.2*mean_dist);
	  id++;
	}
	// grid structured to append center, radius and color label
	vtkSmartPointer<vtkUnstructuredGrid> grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
	grid->SetPoints(points);
	grid->GetPointData()->AddArray(scales);
	grid->GetPointData()->SetActiveScalars("scales"); // !!!to set radius first
	grid->GetPointData()->AddArray(col);

	vtkSmartPointer<vtkCubeSource> cubeSource =  vtkSmartPointer<vtkCubeSource>::New();
	vtkSmartPointer<vtkSphereSource> sphereSource =  vtkSmartPointer<vtkSphereSource>::New();

	vtkSmartPointer<vtkGlyph3D> glyph3D = vtkSmartPointer<vtkGlyph3D>::New();
	glyph3D->SetInputData(grid);
	if(type==CUBE)glyph3D->SetSourceConnection(cubeSource->GetOutputPort());
	else if(type==SPHERE)glyph3D->SetSourceConnection(sphereSource->GetOutputPort());

	vtkSmartPointer<vtkPLYWriter> plyWriter = vtkSmartPointer<vtkPLYWriter>::New();
	plyWriter->SetFileName(savefile.c_str());
	plyWriter->SetInputConnection(glyph3D->GetOutputPort());
	if(!colorless){
		plyWriter->SetLookupTable(lut);
		plyWriter->SetArrayName("col");
		plyWriter->SetColorModeToDefault();
	}
	plyWriter->SetFileTypeToASCII();
	plyWriter->Update();
	plyWriter->Write();


//	 //--------------------- Read and display for verification------------------------------------------------------------------------
//		  vtkSmartPointer<vtkPLYReader> reader =
//		    vtkSmartPointer<vtkPLYReader>::New();
//		  reader->SetFileName(savefile.c_str());
//		  reader->Update();
//
//		  vtkSmartPointer<vtkPolyDataMapper> mappertest =
//		    vtkSmartPointer<vtkPolyDataMapper>::New();
//		  mappertest->SetInputConnection(reader->GetOutputPort());
//		  vtkSmartPointer<vtkActor> actor =    vtkSmartPointer<vtkActor>::New();
//		  actor->SetMapper(mappertest);
//
//		  vtkSmartPointer<vtkRenderer> renderer =
//		    vtkSmartPointer<vtkRenderer>::New();
//		  vtkSmartPointer<vtkRenderWindow> renderWindow =
//		    vtkSmartPointer<vtkRenderWindow>::New();
//		  renderWindow->AddRenderer(renderer);
//		  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
//		    vtkSmartPointer<vtkRenderWindowInteractor>::New();
//		  renderWindowInteractor->SetRenderWindow(renderWindow);
//
//		  renderer->AddActor(actor);
//		  renderer->SetBackground(.3, .6, .3); // Background color green
//
//		  renderWindow->Render();
//		  renderWindowInteractor->Start();
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
	if(argc!=4 && argc!=5 )
	{
		printf("./pcl_preprocessing cloudpath <square/disk/cube/sphere> savepath [cl] \n");
		printf("exemple: ./pcl_preprocessing bunny.pcd square bunny_square.txt cl\n");
		return EXIT_FAILURE;
	}
	std::string cloud_path(argv[1]);
	std::string recon_type(argv[2]);
	std::string save_file(argv[3]);
	if(argc==5 && argv[4]==std::string("cl"))colorless=true;

	printf("loading cloud %s \n",cloud_path.c_str());
	pcl::io::load (cloud_path, *cloud);
	if(!colorless)pcl::io::load (cloud_path, *cloudRGB);
	if(recon_type==std::string("square"))
		regularpolygon2D(getResolution(cloud_path),SQUARE,save_file);
	else if(recon_type==std::string("disk"))
		regularpolygon2D(getResolution(cloud_path),DISK,save_file);
	else if(recon_type==std::string("cube"))
		regularpolygon3D(CUBE,save_file);
	else if(recon_type==std::string("sphere"))
		regularpolygon3D(SPHERE,save_file);

	stop_time=clock();
	cout << "\nExec time: " << (stop_time-start_time)/double(CLOCKS_PER_SEC)*1000<< " ms "<< endl;

	return EXIT_SUCCESS;
}
