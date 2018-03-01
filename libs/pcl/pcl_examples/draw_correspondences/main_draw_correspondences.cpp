#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

using namespace std;
using namespace pcl;

void showHelp(char * program_name) {
	cout << endl;
	cout << "Usage: " << program_name << " source.pcd target.pcd correspondences.txt threshold" << endl;
	cout << "-h: Show this help." << endl;
}

int main(int argc, char** argv) {
	if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help")) {
		showHelp(argv[0]);
	}

	if (argc != 5) {
		showHelp(argv[0]);
		system("pause");
		return -1;
	}

	string source_path = string(argv[1]);
	string target_path = string(argv[2]);
	string corres_path = string(argv[3]);
	float threshold = atof(argv[4]);
	cout << "threshold: " << threshold << endl;

	vector<Correspondence> correspondences;

	ifstream file(corres_path);
	string line;
	while (getline(file, line)) {
		int source, target;
		float distance;
		stringstream ss;
		ss << line;
		ss >> source >> target >> distance;
		if (distance < threshold) {
			cout << "source: " << source << " target: " << target << " dist: " << distance << endl;
			correspondences.push_back(Correspondence(source, target, distance));
		}
	}

	PointCloud<PointXYZ>::Ptr source_cloud(new PointCloud<PointXYZ>());
	if (io::loadPCDFile(source_path, *source_cloud) < 0) {
		cout << "Error loading " << source_path << endl;
		showHelp(argv[0]);
		system("pause");
		return -1;
	}

	PointCloud<PointXYZ>::Ptr target_cloud(new PointCloud<PointXYZ>());
	if (io::loadPCDFile(target_path, *target_cloud) < 0) {
		if (io::loadOBJFile(target_path, *target_cloud) < 0) {
			cout << "Error loading " << target_path << endl;
			showHelp(argv[0]);
			system("pause");
			return -1;
		}
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.rotate(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitY()));
		pcl::transformPointCloud(*target_cloud, *target_cloud, transform);
		transform.setIdentity();
		transform.rotate(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitX()));
		pcl::transformPointCloud(*target_cloud, *target_cloud, transform);

	}


	// Translation
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << 0, 1.0, 0;

	pcl::transformPointCloud(*target_cloud, *target_cloud, transform);

	// Visualization
	pcl::visualization::PCLVisualizer viewer("Extract plane example");

	float bckgr_gray_level = 0.0;  // Black

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255, 0, 0);
	viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color_handler(target_cloud, 0, 255, 0);
	viewer.addPointCloud(target_cloud, target_cloud_color_handler, "target_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud");

	for (int i = 0; i < correspondences.size(); i++) {
		int source = correspondences[i].index_query;
		int target = correspondences[i].index_match;

		viewer.addLine(source_cloud->points[source], target_cloud->points[target], 255, 0, 0, "arrow" + to_string(i));
	}

	viewer.addCoordinateSystem(1.0, "cloud", 0);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);

	viewer.setSize(1280, 1024);  // Visualiser window size

	while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce();
	}

	system("pause");
}