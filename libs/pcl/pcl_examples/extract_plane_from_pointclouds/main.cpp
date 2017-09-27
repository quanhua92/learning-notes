#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

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
		cout << "Error loading " << argv[1] << endl;
		showHelp(argv[0]);
		system("pause");
		return -1;
	}

	// scale up point clouds
	//double N = 10;
	//Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	//transform(0, 0) = transform(0, 0) * N;
	//transform(1, 1) = transform(1, 1) * N;
	//transform(2, 2) = transform(2, 2) * N;
	//pcl::transformPointCloud(*source_cloud, *source_cloud, transform);

	// process
	PointCloud<PointXYZ>::Ptr extracted_cloud(new PointCloud<PointXYZ>());
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(source_cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.25, 0.26); //  0.1 = 10cm
	pass.filter(*extracted_cloud);

	PointCloud<PointXYZ>::Ptr reference_cloud(new PointCloud<PointXYZ>());
	for (int i = 0; i < 15; i++) {
		for (int j = 0; j < 15; j++) {
			PointXYZ t;
			t.x = i * 0.01 + 0.35; // 0.01 = 1cm
			t.y = j * 0.01 + 0.25;
			t.z = 0.25;
			reference_cloud->points.push_back(t);
		}
	}

	// Visualization
	pcl::visualization::PCLVisualizer viewer("Extract plane example");
	int v1(0);
	int v2(1);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;
	viewer.addText("Original point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "info_1", v1);
	viewer.addText("Extracted point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "info_2", v2);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255, 255, 255);
	viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original_cloud", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(extracted_cloud, 230, 20, 20); // Red
	viewer.addPointCloud(extracted_cloud, transformed_cloud_color_handler, "extracted_cloud", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ref_cloud_color_handler(extracted_cloud, 230, 230, 230); // Red
	viewer.addPointCloud(reference_cloud, ref_cloud_color_handler, "ref_cloud", v2);

	viewer.addCoordinateSystem(1.0, "cloud", 0);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "extracted_cloud");
	// Set camera position and orientation
	//viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setCameraPosition(0.05, 0.05, 0.3, 0, 0, -1);

	viewer.setSize(1280, 1024);  // Visualiser window size

	while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce();
	}

	return 0;
}
