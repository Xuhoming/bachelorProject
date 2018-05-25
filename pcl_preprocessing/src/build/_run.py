from pprint import pprint
import os


# contents = ['amphoriskos', 'head', 'longdress', 'loot', 'mask', 'queen', 'redandblack', 'shiva']
contents = ['soldier']
shapes = ['disk', 'cube', 'sphere', 'square']
# shapes = ['sphere']

for content in contents:
	for shape in shapes:
		command = './pcl_preprocessing '+content+'.ply '+shape+' '+content+'_'+shape+'_sc2.ply nl'
		os.system(command)

		command2 = 'pcl_ply2vtk '+content+'_'+shape+'_sc2.ply '+content+'_'+shape+'_sc2.vtk'
		os.system(command2)



