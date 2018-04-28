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


#include <vtkSTLWriter.h>
#include <vtkSTLReader.h>

int start_time,stop_time;

bool colorless=false;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB (new pcl::PointCloud<pcl::PointXYZRGB>);

void regularpolygon2D(double density,int numbSide,std::string &savefile)
{
	ofstream writtingFile (savefile.c_str());

	if(numbSide==4)
	writtingFile <<"#type: square \n";
	else
	writtingFile <<"#type: disk\n";
	writtingFile <<"#size: "<< cloud->points.size()<<"\n";
	writtingFile << "#Fields: x y z r g b X11 X12 X13 X21 X22 X23 X31 X32 X33\n";
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
	double scale;
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

		writtingFile <<std::setprecision (17)<< cloud->points[i].x<<" "<< cloud->points[i].y<<" " <<cloud->points[i].z<<" ";
		if(colorless)writtingFile << 1 <<" "<< 1 <<" "<< 1;
		else writtingFile <<std::setprecision (17)<< cloudRGB->points[i].r/255.0<<" "<<cloudRGB->points[i].g/255.0<<" "<<cloudRGB->points[i].b/255.0;
		for (int j=0;j<9;j++)writtingFile <<std::setprecision (17)<<" "<<rotation[j];
		writtingFile<< std::endl;
	}
	writtingFile.close();
//
//	vtkSmartPointer<vtkUnstructuredGrid> grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
//	grid->SetPoints(points);
//	grid->GetPointData()->SetTensors(tensors);
//
//
//	// Create a circle
//	vtkSmartPointer<vtkRegularPolygonSource> polygonSource =  vtkSmartPointer<vtkRegularPolygonSource>::New();
//
//	polygonSource->SetNumberOfSides(20);
//	polygonSource->SetRadius(1);
//	polygonSource->GeneratePolylineOff();
//	polygonSource->Update();
//
//	vtkSmartPointer<vtkTensorGlyph> tensorGlyph = vtkSmartPointer<vtkTensorGlyph>::New();
//	tensorGlyph->SetInputData(grid);
//	tensorGlyph->SetSourceConnection(polygonSource->GetOutputPort());
//	tensorGlyph->ColorGlyphsOff();
//	tensorGlyph->ThreeGlyphsOff();
//
//	tensorGlyph->ExtractEigenvaluesOff();
//
//	tensorGlyph->SymmetricOff();
//	tensorGlyph->Update();
//
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
//


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
		regularpolygon2D(getResolution(cloud_path),4,save_file);
	else if(recon_type==std::string("disk"))
		regularpolygon2D(getResolution(cloud_path),20,save_file);
	stop_time=clock();
	cout << "\nExec time: " << (stop_time-start_time)/double(CLOCKS_PER_SEC)*1000<< " ms "<< endl;

	return EXIT_SUCCESS;
}
