#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
using namespace cv;
using namespace std;
using namespace pcl;

//>> > import bmesh
//>> > b = bmesh.from_edit_mesh(bpy.context.object.data)
//>> >[v.index for v in b.verts if v.select]

int IMG_SIZE = 128;
float MM_PER_M = 1000;
float MAX_Z = 2.5;
float MAX_Y = 2.5;
float CAMERA_X = 1.0;
int NUM_OF_BINS = 20;
bool do_visualize = true;

vector<int> SELECTED_LANDMARKS =
{
	982, 1369, 1381, 1386, 1394, 1403, 1405, 1432, 1506, 1520, 1534, 1695, 1746, 3432, 3466,
	3500, 3754, 4072, 4081, 4082, 4170, 4328, 4330, 4331, 4348, 4441, 4480, 4482, 4491, 4536,
	4548, 4613, 4690, 4719, 4748, 4757, 5134, 5148, 5319, 7011, 8060, 8068, 8080, 8086, 8130,
	8161, 8186, 8203, 8209, 9981, 10134, 10424, 10431, 10435, 10541, 10729, 10744, 10812, 10956,
	10958, 10965, 10978, 10979, 11052, 11079, 11109, 11147, 11193, 11291, 11325, 11348, 11365, 11375, 11936 };

void showHelp(char * program_name) {
	cout << endl;
	cout << "Usage: " << program_name << " cloud_filename.pcd" << endl;
	cout << "-h: Show this help." << endl;
}

Vec3b getBinColor(int bin, int num_bin) {
	int color_step = int(255 / num_bin);
	Mat rgb;
	Mat hsv(1, 1, CV_8UC3, Scalar(color_step * bin, 255, 255));
	cvtColor(hsv, rgb, CV_HSV2BGR);
	Vec3b color;
	color[0] = rgb.data[0]; color[1] = rgb.data[1]; color[2] = rgb.data[2];

	return color;
}

void createSegmentationMap(cv::Mat &depthImage, cv::Mat &horizontal, cv::Mat &vertical) {
	double min;
	double max;
	cv::minMaxIdx(depthImage, &min, &max);
	cv::Mat adjMap;
	cv::convertScaleAbs(depthImage, adjMap, 255 / max);

	vector<Point2i> locations;
	findNonZero(adjMap, locations);

	int bottom = 0, right = 0, left = depthImage.cols, top = depthImage.rows - 1;

	for (int i = 0; i < locations.size(); i++) {
		int x = locations[i].x;
		int y = locations[i].y;
		if (x < left) left = x;
		if (x > right) right = x;
		if (y < top) top = y;
		if (y > bottom) bottom = y;
	}

	horizontal = Mat::zeros(Size(depthImage.cols, depthImage.rows), CV_8UC1);
	vertical = Mat::zeros(Size(depthImage.cols, depthImage.rows), CV_8UC1);

	int h_step = round((right - left) / NUM_OF_BINS);
	int v_step = round((bottom - top) / NUM_OF_BINS);

	for (int i = 0; i < locations.size(); i++) {
		int x = locations[i].x;
		int y = locations[i].y;

		int h_bin = int((x - left) / h_step) + 1; //  [1, bins]

		int v_bin = int((y - top) / v_step) + 1; // [1, bins]

		horizontal.at<char>(y, x) = h_bin;
		vertical.at<char>(y, x) = v_bin;
	}

	/// DEBUG
	if (do_visualize) {
		Mat img_rgb;

		cvtColor(adjMap, img_rgb, cv::COLOR_GRAY2RGB);

		rectangle(img_rgb, Rect(left, top, right - left, bottom - top), Scalar(255, 255, 0));
		cv::imshow("img_rgb", img_rgb);

		Mat horizontal_color = Mat::zeros(Size(depthImage.cols, depthImage.rows), CV_8UC3);
		Mat vertical_color = Mat::zeros(Size(depthImage.cols, depthImage.rows), CV_8UC3);

		for (int i = 0; i < depthImage.cols; i++) {
			for (int j = 0; j < depthImage.rows; j++) {
				int h_bin = int(horizontal.at<char>(j, i));
				int v_bin = int(vertical.at<char>(j, i));


				if (h_bin > 0) {


					horizontal_color.at<Vec3b>(j, i) = getBinColor(h_bin, NUM_OF_BINS);
				}
				if (v_bin > 0) {
					vertical_color.at<Vec3b>(j, i) = getBinColor(v_bin, NUM_OF_BINS);
				}
			}
		}
		cv::imshow("h_color", horizontal_color);
		cv::imshow("v_color", vertical_color);

		//waitKey(0);
	}
}


cv::Mat getScanDepthMap(pcl::PointCloud<pcl::PointXYZ> &cloud, cv::Mat &img_depth, cv::Mat &index_map) {
	//int IMG_SIZE = 256;

	img_depth = Mat::zeros(IMG_SIZE, IMG_SIZE, CV_16UC1);
	index_map = Mat::zeros(IMG_SIZE, IMG_SIZE, CV_32FC1);

	for (int i = 0; i < cloud.points.size(); i++) {
		float depth_value = (CAMERA_X - cloud.points[i].x) * MM_PER_M; // convert m to mm
		float z = cloud.points[i].z;
		float y = cloud.points[i].y;

		int img_x = 0, img_y = 0;

		img_x = round(float(IMG_SIZE - 1) * (MAX_Y - y) / MAX_Y);
		img_y = round(float(IMG_SIZE - 1) * (MAX_Z - z) / MAX_Z);

		float current_depth_value = img_depth.at<unsigned short>(img_y, img_x);
		if (current_depth_value == 0 || current_depth_value > depth_value) {
			img_depth.at<unsigned short>(img_y, img_x) = depth_value;
			index_map.at<float>(img_y, img_x) = i;
		}
	}

	Mat img_rgb;

	double min;
	double max;
	cv::minMaxIdx(img_depth, &min, &max);
	cv::Mat adjMap;
	cv::convertScaleAbs(img_depth, adjMap, 255 / max);
	cvtColor(adjMap, img_rgb, cv::COLOR_GRAY2RGB);

	return img_rgb;
}

cv::Mat getDepthMap(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ> &sparse_cloud, cv::Mat &img_depth, vector<Point2f> &source_points) {
	//int IMG_SIZE = 256;

	img_depth = Mat::zeros(IMG_SIZE, IMG_SIZE, CV_16UC1);

	for (int i = 0; i < cloud.points.size(); i++) {
		float depth_value = (CAMERA_X - cloud.points[i].x) * MM_PER_M; // convert m to mm
		float z = cloud.points[i].z;
		float y = cloud.points[i].y;

		int img_x = 0, img_y = 0;

		img_x = round(float(IMG_SIZE - 1) * (MAX_Y - y) / MAX_Y);
		img_y = round(float(IMG_SIZE - 1) * (MAX_Z - z) / MAX_Z);

		float current_depth_value = img_depth.at<unsigned short>(img_y, img_x);
		if (current_depth_value == 0 || current_depth_value > depth_value) {
			img_depth.at<unsigned short>(img_y, img_x) = depth_value;
		}
	}

	Mat img_rgb;

	double min;
	double max;
	cv::minMaxIdx(img_depth, &min, &max);
	cv::Mat adjMap;
	cv::convertScaleAbs(img_depth, adjMap, 255 / max);
	cvtColor(adjMap, img_rgb, cv::COLOR_GRAY2RGB);

	// Draw landmarks
	for (int landmark_index = 0; landmark_index < SELECTED_LANDMARKS.size(); landmark_index++) {
		int i = SELECTED_LANDMARKS[landmark_index];
		float z = sparse_cloud.points[i].z;
		float y = sparse_cloud.points[i].y;

		int img_x = 0, img_y = 0;

		img_x = round(float(IMG_SIZE - 1) * (MAX_Y - y) / MAX_Y);
		img_y = round(float(IMG_SIZE - 1) * (MAX_Z - z) / MAX_Z);

		source_points.push_back(Point2f(img_x, img_y));
	}
	return img_rgb;
}
PointCloud<PointXYZ>::Ptr noiseFiltering(PointCloud<PointXYZ>::Ptr inputCloud, int neighbors) {
	PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>);

	pcl::RadiusOutlierRemoval<PointXYZ> outrem;
	// build the filter
	outrem.setInputCloud(inputCloud);
	outrem.setRadiusSearch(0.01);
	outrem.setMinNeighborsInRadius(neighbors);
	// apply filter
	outrem.filter(*cloud_filtered);
	return cloud_filtered;
}
PointCloud<PointXYZ>::Ptr downsampleCloud(PointCloud<PointXYZ>::Ptr inputCloud, double voxel_size = 0.01) {
	PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>);
	pcl::VoxelGrid<PointXYZ> downsampler;
	downsampler.setInputCloud(inputCloud);
	downsampler.setLeafSize(voxel_size, voxel_size, voxel_size);
	downsampler.filter(*cloud_filtered);
	return cloud_filtered;
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

	// read mean shape
	PointCloud<PointXYZ>::Ptr mean_sparse_cloud(new PointCloud<PointXYZ>());
	//PointCloud<PointXYZ>::Ptr mean_cloud(new PointCloud<PointXYZ>());
	//if (io::loadPCDFile("D:\\mhmodels_old\\mean_shape\\mean_shape_hd.pcd", *mean_cloud) < 0) {
	//	cout << "Error loading mean_cloud" << endl;
	//	showHelp(argv[0]);
	//	system("pause");
	//	return -1;
	//}

	if (io::loadPCDFile("D:\\mhmodels_old\\mean_shape\\mean_shape.pcd", *mean_sparse_cloud) < 0) {
		cout << "Error loading mean_cloud" << endl;
		showHelp(argv[0]);
		system("pause");
		return -1;
	}
	cout << "mean_shape points: " << mean_sparse_cloud->points.size() << endl;

	// read scan cloud
	PointCloud<PointXYZ>::Ptr scan_cloud(new PointCloud<PointXYZ>());
	if (io::loadPCDFile(string(argv[1]), *scan_cloud) < 0) {
		cout << "Error loading scan_cloud" << endl;
		showHelp(argv[0]);
		system("pause");
		return -1;
	}

	//// Translation
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	// rotate scan -> mean
	transform.rotate(Eigen::AngleAxisf(+M_PI / 2, Eigen::Vector3f::UnitY()));
	transformPointCloud(*scan_cloud, *scan_cloud, transform);
	transform.setIdentity();
	transform.translation() << -0.5, 0, 0.5;
	transformPointCloud(*scan_cloud, *scan_cloud, transform);
	transform.rotate(Eigen::AngleAxisf(+M_PI, Eigen::Vector3f::UnitZ()));
	transformPointCloud(*scan_cloud, *scan_cloud, transform);

	// move centroid
	transform.setIdentity();
	transform.translation() << 0, 1.0, 0.1;
	pcl::transformPointCloud(*mean_sparse_cloud, *mean_sparse_cloud, transform);

	// move centroid
	transform.setIdentity();
	transform.translation() << 0.5, 1.0, 0.1;
	pcl::transformPointCloud(*scan_cloud, *scan_cloud, transform);

	// down sample cloud
	PointCloud<PointXYZ>::Ptr down_scan_cloud = downsampleCloud(scan_cloud, 0.025);

	// perform icp 
	pcl::IterativeClosestPoint<PointXYZ, PointXYZ> icp;
	icp.setMaximumIterations(10);
	icp.setInputSource(down_scan_cloud);
	icp.setInputTarget(mean_sparse_cloud);
	icp.align(*down_scan_cloud);

	pcl::transformPointCloud(*scan_cloud, *scan_cloud, icp.getFinalTransformation());
	scan_cloud = noiseFiltering(scan_cloud, 2);
	Mat scan_depth_img;
	Mat scan_index_img;
	getScanDepthMap(*scan_cloud, scan_depth_img, scan_index_img);


	double min;
	double max;
	cv::minMaxIdx(scan_depth_img, &min, &max);
	cv::Mat adjMap;
	cv::convertScaleAbs(scan_depth_img, adjMap, 255 / max);
	imwrite("D:\\scan_depth_map.png", scan_depth_img);
	imwrite("D:\\scan_index_map.png", scan_index_img);

	FileStorage fs;
	fs.open("D:\\scan_depth_index.yml.gz", FileStorage::WRITE);
	fs.write("scan_index_map", scan_index_img);
	fs.write("scan_depth_map", scan_depth_img);
	fs.release();

	imshow("scan depth map", adjMap);
	waitKey(0);
	//imwrite("D:\\depth_img.png", depth_img);

	//// Visualization
	//pcl::visualization::PCLVisualizer viewer("Extract plane example");
	//int v1(0);
	//int v2(1);
	//viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	//viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

	//float bckgr_gray_level = 0.0;  // Black

	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(mean_sparse_cloud, 255, 255, 255);
	//viewer.addPointCloud(mean_sparse_cloud, source_cloud_color_handler, "mean_sparse_cloud", v1);
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "mean_sparse_cloud");

	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sparse_cloud_color_handler(scan_cloud, 255, 255, 255);
	//viewer.addPointCloud(scan_cloud, sparse_cloud_color_handler, "scan_cloud", v2);
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scan_cloud");

	//viewer.addCoordinateSystem(1.0, "cloud", 0);
	//viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	//viewer.setCameraPosition(0.05, 0.05, 0.3, 0, 0, -1);

	//viewer.setSize(1280, 1024);  // Visualiser window size

	//while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
	//	viewer.spinOnce();
	//}

	system("pause");

	return 0;
}
