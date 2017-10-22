// Source: http://pointclouds.org/documentation/tutorials/matrix_transform.php#matrix-transform

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;

void showHelp(char * program_name) {
	cout << endl;
	cout << "Usage: " << program_name << " cloud_filename.pcd" << endl;
	cout << "-h: Show this help." << endl;
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
	if (io::loadPCDFile(argv[1], *source_cloud) < 0) {
	//if (io::loadOBJFile("D:\\datasets\\makehuman-models\\model_001.obj", *source_cloud) < 0) {
		cout << "Error loading " << argv[1] << endl;
		showHelp(argv[0]);
		system("pause");
		return -1;
	}

	// METHOD #1: Using a Matrix4f
	// This is the "manual" method, perfect to understand but error prone !
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

	// Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
	float theta = M_PI / 4; // The angle of rotation in radians
	transform_1(0, 0) = cos(theta);
	transform_1(0, 1) = -sin(theta);
	transform_1(1, 0) = sin(theta);
	transform_1(1, 1) = cos(theta);
	//    (row, column)
	// Define a translation of 0.5 meters on the x axis.
	transform_1(0, 3) = 0.5;

	// Print the transformation
	cout << "Method #1: using a Matrix4f" << endl;
	cout << transform_1 << std::endl;

	// METHOD #2: Using a Affine3f
	// This method is easier and less error prone
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	transform_2.translation() << 0.5, 0, 0;
	transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
	cout << "Method #2: using an Affine3f" << endl;
	cout << transform_2.matrix() << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// You can either apply transform_1 or transform_2; they are the same
	pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);

	// Visualization
	printf("\nPoint cloud colors :  white  = original point cloud\n"
		   "                        red  = transformed point cloud\n");
	pcl::visualization::PCLVisualizer viewer("Matrix transformation example");
	int v1(0);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.setBackgroundColor(0, 0, 0, v1);
	viewer.addText("Radius: 0.01", 10, 10, "v1 text", v1);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> rgb(source_cloud);
	viewer.addPointCloud<pcl::PointXYZ>(source_cloud, rgb, "sample cloud1", v1);

	int v2(0);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer.setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer.addText("Radius: 0.1", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(source_cloud, 0, 255, 0);
	viewer.addPointCloud<pcl::PointXYZ>(source_cloud, single_color, "sample cloud2", v2);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255, 255, 255);
	viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed_cloud, 230, 20, 20); // Red
	viewer.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

	viewer.addCoordinateSystem(1.0, "cloud", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");

	while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce();
	}

	return 0;
}
