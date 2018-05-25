#include <pcl/io/auto_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d_omp.h>
#include <vtkFloatArray.h>
#include <vtkUnstructuredGrid.h>
#include <vtkLookupTable.h>
#include <vtkUnstructuredGrid.h>
#include <vtkTensorGlyph.h>
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
#include <vtkDiskSource.h>
#include <vtkGlyph3D.h>


#include <vtkPLYWriter.h>
#include <vtkPLYReader.h>


//------------------ Initializations -------------------------------
#define NUMB_NEIGH_SEARCH 10
#define NORMAL_SEARCH_NUMBER 10
#define NORMAL_SEARCH_RADIUS_FACTOR 1
#define SHAPE_SCALE_FACTOR 1 // or 2


enum type{SQUARE,DISK,CUBE,SPHERE};
bool colorless=false;
bool normalless=false;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudRGBNormal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

double density;
double std_density;
double factor = 2;


void regularpolygon2D(int type,std::string &savefile)
{

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	tree->setInputCloud(cloud);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	if(normalless)
	{
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud (cloud);
		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).

		ne.setSearchMethod (tree);
		// Output datasets


//------------------------------------------------- Type of search neighbor for normal estimation ----------------------------------------------------
	//	ne.setRadiusSearch (NORMAL_SEARCH_RADIUS_FACTOR*density);
		ne.setKSearch(NORMAL_SEARCH_NUMBER);
//-----------------------------------------------------------------------------------------------------------
		// Compute the features
		ne.compute (*cloud_normals);
	}

//	vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();
//	normals->SetName("normals");
//	normals->SetNumberOfComponents(3);

	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

	vtkSmartPointer<vtkFloatArray> col = vtkSmartPointer<vtkFloatArray>::New();
	col->SetName("col");
	vtkSmartPointer<vtkLookupTable> lut = vtkSmartPointer<vtkLookupTable>::New();
	lut->SetNumberOfTableValues(cloud->points.size());
	lut->SetRange(0,cloud->points.size());

	vtkSmartPointer<vtkDoubleArray> tensors = vtkSmartPointer<vtkDoubleArray>::New();
	// tensors->SetNumberOfTuples(3); // WHY?
	tensors->SetNumberOfComponents(9);

	double normal[3];
	double rotAxis[3];
	double theta;

	pcl::PointXYZ searchPoint;
	int NumbNeighbor = NUMB_NEIGH_SEARCH + 1;

	std::vector<int> nearestNeighborId(NumbNeighbor);
	std::vector<float> nearestNeighborDist(NumbNeighbor);
	double max_dist=0, mean_dist=0, min_dist=0;
	double scale,norm;
	double rotation[9];

	// int dump = 0;
	int counter;
	// int counter2 = 0;	


	for(int i=0;i<cloud->points.size();i++)
	{
		col->InsertNextValue(i);
		if(!colorless)lut->SetTableValue(i,cloudRGB->points[i].r/255.0,cloudRGB->points[i].g/255.0,cloudRGB->points[i].b/255.0);
		min_dist = INFINITY;
		max_dist = 0;
		mean_dist = 0;
		
		points->InsertNextPoint(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);
		if(normalless)
		{
			normal[0]=cloud_normals->points[i].normal_x;
			normal[1]=cloud_normals->points[i].normal_y;
			normal[2]=cloud_normals->points[i].normal_z;
		}
		else
		{
			normal[0]=cloudRGBNormal->points[i].normal_x;
			normal[1]=cloudRGBNormal->points[i].normal_y;
			normal[2]=cloudRGBNormal->points[i].normal_z;
		}

		//normal cross e_z product 
		rotAxis[0] = normal[1] ;
		rotAxis[1] = -normal[0] ;
		rotAxis[2] = 0;
		
		norm = sqrt(rotAxis[0]*rotAxis[0]+rotAxis[1]*rotAxis[1]+rotAxis[2]*rotAxis[2]);
		rotAxis[0] =rotAxis[0]/norm;
		rotAxis[1] =rotAxis[1]/norm;
		rotAxis[2] =rotAxis[2]/norm;

		theta=std::acos(normal[2]);

		searchPoint.x=cloud->points[i].x;
		searchPoint.y=cloud->points[i].y;
		searchPoint.z=cloud->points[i].z;


		if ( tree->nearestKSearch (searchPoint, NumbNeighbor, nearestNeighborId, nearestNeighborDist) > 0 )
		{	
			  counter = 0;
			  
			  for (size_t i = 0; i < nearestNeighborId.size (); ++i)
			  {		
			  	if (nearestNeighborDist[i] != 0)
			  	{
			  		counter = counter+1;
			  		// counter2 = counter2+1;

			  		if(sqrt(nearestNeighborDist[i])>max_dist)max_dist=sqrt(nearestNeighborDist[i]);
			  		if(sqrt(nearestNeighborDist[i])<min_dist)min_dist=sqrt(nearestNeighborDist[i]);
				  	mean_dist+=sqrt(nearestNeighborDist[i]);
			  	}
				
			  }
			  mean_dist/=(counter);
		}

		// // Keep size of primitive shapes within limits based on the infinity-norm
		// if(max_dist<density)
		// 	scale=density;
		// else if(max_dist > density + 2*std_density)
		// 	scale=2*density;
		// else
		// 	scale=max_dist;

		// If "outliers" are identified, set the global average
		if(mean_dist > (density + factor*std_density))
			scale = density;
		else if(mean_dist < (density - factor*std_density))
			scale = density;
		else
			scale = mean_dist;


		// Apply tuning factor
		scale = SHAPE_SCALE_FACTOR*scale;
		
		// // Avoid amplifying the shapes of points that are sparse
		// if (mean_dist < density)
	 //  	{
	 //  		scale=density;
	 //  	}
	 //  	else if (mean_dist > 2*density)
	 //  	{
	 //  		scale=2*density;	
	 //  	}
	 //  	else
	 //  	{
	 //  		scale=mean_dist;
	 //  	}
	 

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

	// cout << dump << endl;
	// cout << cloud->points.size() << endl;
	// cout << counter2 << endl;

	vtkSmartPointer<vtkUnstructuredGrid> grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
	grid->SetPoints(points);
	grid->GetPointData()->SetTensors(tensors);
	grid->GetPointData()->AddArray(col);
	grid->GetPointData()->SetActiveScalars("col");

	vtkSmartPointer<vtkTensorGlyph> tensorGlyph = vtkSmartPointer<vtkTensorGlyph>::New();
	tensorGlyph->SetInputData(grid);

	// Create a polygon
	if(type==SQUARE)
	{
		vtkSmartPointer<vtkRegularPolygonSource> polygonSource =  vtkSmartPointer<vtkRegularPolygonSource>::New();
		polygonSource->SetNumberOfSides(4);
		polygonSource->GeneratePolylineOff();
		polygonSource->SetRadius(0.658);	
		polygonSource->Update();
		tensorGlyph->SetSourceConnection(polygonSource->GetOutputPort());

	}
	else if(type==DISK)
	{	
		// vtkSmartPointer<vtkDiskSource> diskSource =  vtkSmartPointer<vtkDiskSource>::New();
		// // polygonSource->SetNumberOfSides(8);
		// diskSource->SetInnerRadius(0);
		// diskSource->SetOuterRadius(1);
		// diskSource->SetCircumferentialResolution(8);
		// diskSource->Update();
		// tensorGlyph->SetSourceConnection(diskSource->GetOutputPort());

		vtkSmartPointer<vtkRegularPolygonSource> polygonSource =  vtkSmartPointer<vtkRegularPolygonSource>::New();
		polygonSource->SetNumberOfSides(16);
		// polygonSource->SetRadius(2*0.3714);
		polygonSource->SetRadius(0.564);
		polygonSource->GeneratePolylineOff();
		polygonSource->Update();
		tensorGlyph->SetSourceConnection(polygonSource->GetOutputPort());
	}

	tensorGlyph->ColorGlyphsOn();
	tensorGlyph->ThreeGlyphsOff();
	tensorGlyph->SetColorModeToScalars();
	tensorGlyph->ExtractEigenvaluesOff();
	tensorGlyph->SymmetricOff();
	tensorGlyph->Update();


	//--------------------- Save the file ------------------------------------------------------------------------
	vtkSmartPointer<vtkPLYWriter> plyWriter = vtkSmartPointer<vtkPLYWriter>::New();
	plyWriter->SetFileName(savefile.c_str());
	plyWriter->SetInputConnection(tensorGlyph->GetOutputPort());
	if(!colorless){
		plyWriter->SetLookupTable(lut);
		plyWriter->SetArrayName("col");
		plyWriter->SetColorModeToDefault();
	}
	plyWriter->SetFileTypeToBinary();
	// plyWriter->SetFileTypeToASCII();
	plyWriter->Update();
	plyWriter->Write();

	
	//--------------------- Read and display for verification ------------------------------------------------------------------------
	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
	reader->SetFileName(savefile.c_str());
	reader->Update();

	vtkSmartPointer<vtkPolyDataMapper> mappertest =
	vtkSmartPointer<vtkPolyDataMapper>::New();
	mappertest->SetInputConnection(reader->GetOutputPort());

	vtkSmartPointer<vtkActor> actor =    vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mappertest);
	if(!colorless)actor->GetProperty()->LightingOff();

	vtkSmartPointer<vtkRenderer> renderer =
	vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow =
	vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
	vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =  
	vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	style->SetInteractor(renderWindowInteractor);
	renderWindowInteractor->SetInteractorStyle(style);
	style->SetCurrentRenderer(renderer);

	renderer->AddActor(actor);
	renderer->SetBackground(0,0,0);

	renderWindow->Render();
	renderWindowInteractor->Start();

}

void regularpolygon3D(int type,std::string &savefile)
{
	double mean_dist=0,max_dist=0,scaling=0;
	double min_dist;

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
	int NumbNeighbor = NUMB_NEIGH_SEARCH + 1;

	std::vector<int> nearestNeighborId(NumbNeighbor);
	std::vector<float> nearestNeighborDist(NumbNeighbor);
	
	// int dump = 0;
	int counter;
	// int counter2 = 0;

	for(int i=0;i<cloud->points.size();i++)
	{
	  	min_dist = INFINITY;
	  	max_dist=0;
	  	mean_dist=0;

	  	points->InsertNextPoint(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);
	  	searchPoint.x=cloud->points[i].x;
	  	searchPoint.y=cloud->points[i].y;
	  	searchPoint.z=cloud->points[i].z;

	 	if ( kdtree.nearestKSearch (searchPoint, NumbNeighbor, nearestNeighborId, nearestNeighborDist) > 0 )
	  	{	
	  		counter = 0;
		  	for (size_t j = 0; j < nearestNeighborId.size (); ++j)
		  	{	
		  		if (nearestNeighborDist[j] != 0)
			 	{	
			 		counter = counter+1;
			 		// counter2 = counter2+1;

			  		if(sqrt(nearestNeighborDist[j])>max_dist)max_dist=sqrt(nearestNeighborDist[j]);
			  		if(sqrt(nearestNeighborDist[j])<min_dist)min_dist=sqrt(nearestNeighborDist[j]);
			  		mean_dist+=sqrt(nearestNeighborDist[j]);
			  	}
		  	}
		  	mean_dist/=(counter);
	  	}
	  	
	  	if(colorless)
			lut->SetTableValue(i,1,1,1);
	  	else
	  		lut->SetTableValue(i,cloudRGB->points[i].r/255.0,cloudRGB->points[i].g/255.0,cloudRGB->points[i].b/255.0);
	  	
	  	col->InsertNextValue(i);

	  // 	// Keep size of primitive shapes within limits based on the infinity-norm
	  // 	if(max_dist<density)
			// scaling=density;
	  // 	else if(max_dist>2*density)
			// scaling=2*density;
	  // 	else
			// scaling=max_dist;

	  	// If "outliers" are identified, set the global average
	  	if(mean_dist > (density + factor*std_density))
			scaling = density;
		else if(mean_dist < (density - factor*std_density))
			scaling = density;
		else
			scaling = mean_dist;

		// Apply tuning factor
		scaling = SHAPE_SCALE_FACTOR*scaling;

	  	// // Avoid amplifying the shapes of points that are sparse
	  	// if (mean_dist < density)
	  	// {
	  	// 	scaling=density;
	  	// }
	  	// else if (mean_dist > 2*density)
	  	// {
	  	// 	scaling=2*density;	
	  	// }
	  	// else
	  	// {
	  	// 	scaling=mean_dist;
	  	// }
	  	
	  	scales->InsertNextValue(scaling);
	}

	// cout << dump << endl;
	// cout << cloud->points.size() << endl;
	// cout << counter2 << endl;

	// grid structured
	vtkSmartPointer<vtkUnstructuredGrid> grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
	grid->SetPoints(points);
	grid->GetPointData()->AddArray(scales);
	grid->GetPointData()->SetActiveScalars("scales"); // !!!to set radius first
	grid->GetPointData()->AddArray(col);

	vtkSmartPointer<vtkGlyph3D> glyph3D = vtkSmartPointer<vtkGlyph3D>::New();
	glyph3D->SetInputData(grid);

	if(type==CUBE)
	{
		vtkSmartPointer<vtkCubeSource> cubeSource =  vtkSmartPointer<vtkCubeSource>::New();	
		glyph3D->SetSourceConnection(cubeSource->GetOutputPort());
	}
	else if (type==SPHERE)
	{
		vtkSmartPointer<vtkSphereSource> sphereSource =  vtkSmartPointer<vtkSphereSource>::New();
		sphereSource->SetThetaResolution(6);
		sphereSource->SetPhiResolution(6);
		// sphereSource->SetRadius(0.564);
		sphereSource->SetRadius(2*0.3714);	
		glyph3D->SetSourceConnection(sphereSource->GetOutputPort());
	}


	//--------------------- Save the file ------------------------------------------------------------------------
	vtkSmartPointer<vtkPLYWriter> plyWriter = vtkSmartPointer<vtkPLYWriter>::New();
	plyWriter->SetFileName(savefile.c_str());
	plyWriter->SetInputConnection(glyph3D->GetOutputPort());
	if(!colorless)
	{
		plyWriter->SetLookupTable(lut);
		plyWriter->SetArrayName("col");
		plyWriter->SetColorModeToDefault();
	}
	plyWriter->SetFileTypeToBinary();
	// plyWriter->SetFileTypeToASCII();
	plyWriter->Update();
	plyWriter->Write();


	 //--------------------- Read and display for verification ------------------------------------------------------------------------
	vtkSmartPointer<vtkPLYReader> reader =
	vtkSmartPointer<vtkPLYReader>::New();
	reader->SetFileName(savefile.c_str());
	reader->Update();

	vtkSmartPointer<vtkPolyDataMapper> mappertest =
	vtkSmartPointer<vtkPolyDataMapper>::New();
	mappertest->SetInputConnection(reader->GetOutputPort());
	vtkSmartPointer<vtkActor> actor =    vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mappertest);
	if(!colorless)actor->GetProperty()->LightingOff();
	vtkSmartPointer<vtkRenderer> renderer =
	vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow =
	vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
	vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =  
	vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	style->SetInteractor(renderWindowInteractor);
	renderWindowInteractor->SetInteractorStyle(style);
	style->SetCurrentRenderer(renderer);

	renderer->AddActor(actor);
	renderer->SetBackground(0, 0, 0); // Background color green

	renderWindow->Render();
	renderWindowInteractor->Start();
}


double getResolution(std::string &filename)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	
	if (pcl::io::load (filename, *cloud))
	{
		return false;
	}
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	pcl::PointXYZ searchPoint;

	std::vector<int> nearestNeighborId(NUMB_NEIGH_SEARCH);
	std::vector<float> nearestNeighborDist(NUMB_NEIGH_SEARCH);

	double tmpDist; 
    double min_global_NN = INFINITY;
    double max_global_NN = 0;
    double global_kNN_mean = 0;
    double local_kNN_mean = 0;
    double global_kNN_std = 0;
    double global_kNN_var = 0;
	int numPoints = 0;
    int counter = 0;

    // double min_neighbor = INFINITY;
    // double max_neighbor = 0;
    // double mean_neighbor = 0;
    // int numPoints = 0;
    // 
	// 
  	// for(pcl::PointCloud<pcl::PointXYZ>::iterator it_vox = cloud->begin();it_vox != cloud->end(); it_vox++)
  	// {
	 //  	searchPoint.x=it_vox->x;
	 //  	searchPoint.y=it_vox->y;
	 //  	searchPoint.z=it_vox->z;
	// 
		// if ( kdtree.nearestKSearch (searchPoint, 2, nearestNeighborId, nearestNeighborDist) > 0 )
		// {
		// 	for (size_t k = 0; k < nearestNeighborId.size (); ++k)
		//   	{	
		//   		tmpDist = sqrt(nearestNeighborDist[k]);

		//   		if (tmpDist != 0)
		// 	 	{	
		// 	 		if(tmpDist<min_neighbor)
		// 		 	{
		// 		 		min_neighbor=tmpDist;
		// 		 	}
				 
		// 		 	if(tmpDist>max_neighbor)
		// 		 	{
		// 		 		max_neighbor=tmpDist;
		// 		 	}
				 	
		// 		 	mean_neighbor+=tmpDist;

		// 		 	numPoints = numPoints+1;
		// 	 	}
		// 	 }			
	 //  	}
  	// 	}
	// mean_neighbor/=numPoints;

    // Find the mean and the std of distances of a point from its K-nearest neighbors
	for(pcl::PointCloud<pcl::PointXYZ>::iterator it_vox = cloud->begin();it_vox != cloud->end(); it_vox++)
    {
	  	searchPoint.x=it_vox->x;
	  	searchPoint.y=it_vox->y;
	  	searchPoint.z=it_vox->z;

	  	tmpDist = 0; 
	  	
		if ( kdtree.nearestKSearch (searchPoint, NUMB_NEIGH_SEARCH, nearestNeighborId, nearestNeighborDist) > 0 )
		{	
			double dist[NUMB_NEIGH_SEARCH];
    		counter = 0;
			local_kNN_mean = 0;

			for (size_t k = 0; k < nearestNeighborId.size (); ++k)
		  	{	
		  		tmpDist = sqrt(nearestNeighborDist[k]);

		  		if (tmpDist != 0)
			 	{	
			 		if(tmpDist<min_global_NN)
				 	{
				 		min_global_NN=tmpDist;
				 	}
				 
				 	if(tmpDist>max_global_NN)
				 	{
				 		max_global_NN=tmpDist;
				 	}

				 	local_kNN_mean = local_kNN_mean+tmpDist;

				 	dist[k] = tmpDist;
				 	counter = counter+1;
			 	}
			 }

			 local_kNN_mean = local_kNN_mean/counter;

			 double temp_var = 0;
			 for(int n = 0; n < counter; n++ )
			 {
			 	temp_var += (dist[n] - local_kNN_mean) * (dist[n] - local_kNN_mean);
			 }

			 global_kNN_mean = global_kNN_mean+local_kNN_mean;
			 global_kNN_var = temp_var/counter;
			 global_kNN_std = global_kNN_std+sqrt(global_kNN_var);

			 numPoints = numPoints+1;
	  	}
  	}

	density = global_kNN_mean/numPoints;
	std_density = global_kNN_std/numPoints;


	printf("Nearest neighbor distances in original cloud - min: %f, max: %f \n", min_global_NN, max_global_NN);
	// printf("Nearest neighboring distances in original cloud - min: %f, max: %f \n", min_neighbor, max_neighbor);
	printf("Average K-nearest neighbor distances in original cloud - avg: %f, std: %f \n",density, std_density);

	
	return 0;
}


int main(int argc, char ** argv)
{
	if(argc!=4 && argc!=5 )
	{
		printf("./pcl_preprocessing cloudpath <square/disk/cube/sphere> savepath [cl] [nl] \n");
		printf("Example: ./pcl_preprocessing bunny.pcd square bunny_square.txt cl nl\n");
		printf("Example: ./pcl_preprocessing longdress.pcd square longdress_square.txt \n");
		return EXIT_FAILURE;
	}
	std::string cloud_path(argv[1]);
	std::string recon_type(argv[2]);
	std::string save_file(argv[3]);
	if((argc==5 || argc==6) && argv[4]==std::string("cl"))
		colorless=true;
	else if((argc==5 && argv[4]==std::string("nl"))||(argc==6 && argv[5]==std::string("nl")))
		normalless=true;
	else
		printf("The given pointcloud is supposed as a xyzrgbnormal could\n");

	printf("Loading cloud %s \n",cloud_path.c_str());
	pcl::io::load (cloud_path, *cloud);
	if(!colorless)pcl::io::load (cloud_path, *cloudRGB);

	getResolution(cloud_path);

	if(recon_type==std::string("square")){
		if(!normalless)pcl::io::load (cloud_path, *cloudRGBNormal);
		regularpolygon2D(SQUARE,save_file);
	}
	else if(recon_type==std::string("disk")){
		if(!normalless)pcl::io::load (cloud_path, *cloudRGBNormal);
		regularpolygon2D(DISK,save_file);
	}
	else if(recon_type==std::string("cube"))
		regularpolygon3D(CUBE,save_file);
	else if(recon_type==std::string("sphere"))
		regularpolygon3D(SPHERE,save_file);

	return EXIT_SUCCESS;
}
