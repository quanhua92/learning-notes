#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/common/transforms.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkPolyDataMapper.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void showHelp(char * program_name) {
	cout << endl;
	cout << "Usage: " << program_name << " cloud_filename.obj" << endl;
	cout << "-h: Show this help." << endl;
}

void loadOBJinHighRes(char * fileName, PointCloud<PointXYZ>::Ptr source_cloud) {
	// load in high res
	vtkSmartPointer<vtkPolyData> polydata1;
	vtkSmartPointer<vtkOBJReader> readerQuery = vtkSmartPointer<vtkOBJReader>::New();
	readerQuery->SetFileName(fileName);
	readerQuery->Update();
	polydata1 = readerQuery->GetOutput();
	//// polydata1->Update(); // this is not available now
	visualization::PCLVisualizer vis;
	vis.addModelFromPolyData(polydata1, "mesh1", 0);
	vis.setRepresentationToSurfaceForAllActors();

	int tesselated_sphere_level = 1;
	int resolution = 100;

	PointCloud<PointXYZ>::CloudVectorType views_xyz;
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
	std::vector<float> enthropies;
	vis.renderViewTesselatedSphere(resolution, resolution, views_xyz, poses, enthropies, tesselated_sphere_level);

	//take views and fuse them together
	std::vector<PointCloud<PointXYZ>::Ptr> aligned_clouds;

	for (size_t i = 0; i < views_xyz.size(); i++)
	{
		PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
		Eigen::Matrix4f pose_inverse;
		pose_inverse = poses[i].inverse();
		transformPointCloud(views_xyz[i], *cloud, pose_inverse);
		aligned_clouds.push_back(cloud);
	}
	for (size_t i = 0; i < aligned_clouds.size(); i++)
		*source_cloud += *aligned_clouds[i];
}

void readVerticesTxt(string fileName, vector<int> &vertices) {
	ifstream file(fileName.c_str());
	while (!file.eof()) {
		string line;
		file>> line;
		vertices.push_back(stoi(line));
	}
}

int main(int argc, char** argv) {
	if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help")) {
		showHelp(argv[0]);
	}

	if (argc != 2) {
		showHelp(argv[0]);
		system("pause");
		return -1;
	}

	PointCloud<PointXYZ>::Ptr source_cloud(new PointCloud<PointXYZ>());
	PointCloud<PointXYZ>::Ptr source_color(new PointCloud<PointXYZ>());
	// // Load normal
	//if (io::loadOBJFile(argv[1], *source_cloud) < 0) {
	//	cout << "Error loading " << argv[1] << endl;
	//	showHelp(argv[0]);
	//	system("pause");
	//	return -1;
	//}

	// load high res
	loadOBJinHighRes(argv[1], source_cloud);

	// load chest & biceps
	PolygonMesh polygonMesh;
	io::loadOBJFile(argv[1], polygonMesh);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(polygonMesh.cloud, *cloud);

	vector<int> chestVertices;
	vector<int> leftBicepsVertices;
	vector<int> rightBicepsVertices;

	readVerticesTxt("D:\\datasets\\makehuman-models\\model_information\\chest_vertices.txt", chestVertices);
	readVerticesTxt("D:\\datasets\\makehuman-models\\model_information\\left_biceps_vertices.txt", leftBicepsVertices);
	readVerticesTxt("D:\\datasets\\makehuman-models\\model_information\\right_biceps_vertices.txt", rightBicepsVertices);
	
	for (int i = 0; i < polygonMesh.polygons.size(); i++) {
		Vertices vertices = polygonMesh.polygons[i];
		bool foundChest = false, foundLeftBiceps = false, foundRightBiceps = false;
		foundChest = find(chestVertices.begin(), chestVertices.end(), i) != chestVertices.end();
		foundLeftBiceps = find(leftBicepsVertices.begin(), leftBicepsVertices.end(), i) != leftBicepsVertices.end();
		foundRightBiceps = find(rightBicepsVertices.begin(), rightBicepsVertices.end(), i) != rightBicepsVertices.end();

		for (int j = 0; j < vertices.vertices.size(); j++) {
			unsigned int index = vertices.vertices[j];
			PointXYZ p = cloud->points[index];

			if (foundChest || foundLeftBiceps || foundRightBiceps) {
				cout << "polygon i = " << i << endl;
				source_color->points.push_back(p);
				cout << "j = " << j << " " << index << endl;
				cout << cloud->points[index] << endl;
			}
			else {
				//source_cloud->points.push_back(p);
			}
		}
	}

	cout << "vertices count " << polygonMesh.polygons.size() << " " << source_cloud->points.size() << " " << cloud->points.size() << endl;
	// Visualization
	pcl::visualization::PCLVisualizer viewer("Load OBJ file in high res");


	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 120, 120, 120);
	viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color_handler(source_color, 0, 0, 255);
	viewer.addPointCloud(source_color, source_color_handler, "color_cloud");

	viewer.addCoordinateSystem(1.0, "cloud", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");

	while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce();
	}

	return 0;
}
