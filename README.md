# bachelorProject
The codes present in this directory require VTK, PCL and Qt to be compiled. To compile each one go to the src directory of the program you want to compile and run:

	mkdir build
	cd build
	cmake ../src/
	make
The following programs are those which can be useful:

## colored_octree
This program allows you to give a point cloud in pcd format. It will process and show a 3D recontruct object using an octree grid estimation and voxels.

	Syntax is: ./colored_octree pcd_file [resolution] [cl] 
	Example: ./colored_octree longdress.pcd 
	Example: ./colored_octree bunny.pcd 0.01 cl

The parameter cl has to be specify when the input cloud is colorless. If the resolution is not precised, an automatic one will be calculated and used.

## pcl_octree_grid
This program take as input a point cloud in pcd format and return a version of the point cloud corresponding to the estimation using an octree grid of either given or automatically extracted resolution.

	Syntax is: ./pcl_octree_grid input_pcd output_pcd [resolution] [cl] 
	Example: ./pcl_octree_grid longdress.pcd longdress_oct.pcd
	Example: ./pcl_octree_grid bunny.pcd bunny_oct.pcd 0.01 cl

## pcl_preprocessing
This program allow you to generate the ply file corresponding to the reconstruct object obtained by surface splatting. 2D or 3D splat can be selected. 

	Syntax is: ./pcl_preprocessing input_pcd <primitive> output_ply [cl] [nl] 
	with primitive being one of: square/disk/cube/sphere 
	Example: ./pcl_preprocessing longdress.pcd disk longdress_disk.ply nl 
	Example: ./pcl_preprocessing bunny.pcd cube bunny_cube.ply cl nl

As for the parameter cl, nl has to be specify if the input cloud is normal-less.

## ReadPLY and ReadVTK
Both program serve to load two ply or vtk files and show them side by side, principally for comparison purpose. The VTK variant possess the advantage of being able to render content without faces which is useful to compare a point cloud to a surface reconstruction of it.

	Syntax is: ./Read(VTK/PLY) input_vtk1 input_vtk2 [cl] 
	Example: ./ReadVTK longdress_point.vtk longdress_sphere.vtk 
	Example: ./ReadPLY bunny_square.ply bunny_cube.ply cl

## qualityAssessment
This program use VTK and Qt and was used to make the subjective quality assessment test. You need to give as input a text file containing 2 vtk files name per line. To work correctly, the following folders are required to be on the same level as the executable:

	-contents 
	-camera 
	-results 
	-batches

The program will store the choices made for every comparison under results and the camera movement under camera.

	Syntax is: ./qualityAssessment batches/batch_xx.txt

## misc
adaptive_voxel_size has been included inside pcl_pcreprocessing when the chosen primitive is 'cube'



