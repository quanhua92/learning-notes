#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

using namespace std;
using namespace cv;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//double FOCAL_X = 616.9788256385079;
//double FOCAL_Y = 618.2612204459588;
//double CENTER_X = 319.6980207838996;
//double CENTER_Y = 237.2675723659526;
//double FOCAL_X = 595.171;
//double FOCAL_Y = 595.171;
//double CENTER_X = 311.823;
//double CENTER_Y = 250.501;
double FOCAL_X = 603.9133550935715;
double FOCAL_Y = 599.7920816123911;
double CENTER_X = 325.771848677865;
double CENTER_Y = 226.4656848780276;
double MM_PER_M = 1000;
int WIDTH = 640;
int HEIGHT = 480;
double MAX_DEPTH_THRESHOLD = 1.6;
double MIN_DEPTH_THRESHOLD = 0.0;

bool next_iteration = false;

void
print4x4Matrix(const Eigen::Matrix4d & matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void
keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
	void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
}

void matwrite(const string& filename, const Mat& mat)
{
	//https://stackoverflow.com/a/32357875
	ofstream fs(filename, fstream::binary);

	// Header
	int type = mat.type();
	int channels = mat.channels();
	fs.write((char*)&mat.rows, sizeof(int));    // rows
	fs.write((char*)&mat.cols, sizeof(int));    // cols
	fs.write((char*)&type, sizeof(int));        // type
	fs.write((char*)&channels, sizeof(int));    // channels

												// Data
	if (mat.isContinuous())
	{
		fs.write(mat.ptr<char>(0), (mat.dataend - mat.datastart));
	}
	else
	{
		int rowsz = CV_ELEM_SIZE(type) * mat.cols;
		for (int r = 0; r < mat.rows; ++r)
		{
			fs.write(mat.ptr<char>(r), rowsz);
		}
	}
}

Mat matread(const string& filename)
{
	//https://stackoverflow.com/a/32357875
	ifstream fs(filename, fstream::binary);

	// Header
	int rows, cols, type, channels;
	fs.read((char*)&rows, sizeof(int));         // rows
	fs.read((char*)&cols, sizeof(int));         // cols
	fs.read((char*)&type, sizeof(int));         // type
	fs.read((char*)&channels, sizeof(int));     // channels

												// Data
	Mat mat(rows, cols, type);
	fs.read((char*)mat.data, CV_ELEM_SIZE(type) * rows * cols);

	return mat;
}

void convert_to_pointclouds(PointCloudT &pointcloud, cv::Mat rgb_img, cv::Mat depth_img, cv::Mat rot_mat, cv::Mat trans_mat)
{
	trans_mat = trans_mat / 10 / MM_PER_M;

	for (int i = 0; i < rgb_img.rows; i++) {
		for (int j = 0; j < rgb_img.cols; j++) {
			double depth = double(depth_img.at<unsigned short>(i, j));
			//depth = 100;
			if (depth != 0 && depth / MM_PER_M < MAX_DEPTH_THRESHOLD && depth / MM_PER_M > MIN_DEPTH_THRESHOLD) {
				Mat camPoint = Mat::zeros(3, 1, CV_64FC1);
				camPoint.at<double>(0, 0) = (j - CENTER_X) * depth / FOCAL_X / MM_PER_M;
				camPoint.at<double>(1, 0) = (i - CENTER_Y) * depth / FOCAL_Y / MM_PER_M;
				camPoint.at<double>(2, 0) = double(depth) / MM_PER_M;

				// Mat worldPoint = rot_mat * camPoint - rot_mat.t() * trans_mat;
				// Mat worldPoint = rot_mat * camPoint + rot_mat.t() * trans_mat;
				 //Mat worldPoint = camPoint + rot_mat.t() * trans_mat;
				//Mat worldPoint = rot_mat.t() * camPoint - trans_mat;
				Mat worldPoint = rot_mat * camPoint + trans_mat;
				//Mat worldPoint = camPoint;

				PointT point;
				point.x = worldPoint.at<double>(0, 0);
				point.y = worldPoint.at<double>(1, 0);
				point.z = worldPoint.at<double>(2, 0);
				Vec3b rgb = rgb_img.at<Vec3b>(i, j);

				point.b = (uint8_t)rgb[0];
				point.g = (uint8_t)rgb[1];
				point.r = (uint8_t)rgb[2];

				pointcloud.points.push_back(point);
			}
		}
	}
	pointcloud.width = (uint32_t)pointcloud.points.size();
	pointcloud.height = 1;
}

PointCloudT::Ptr downsampleCloud(PointCloudT::Ptr inputCloud, double voxel_size= 0.01)
{
	PointCloudT::Ptr cloud_filtered(new PointCloudT);
	pcl::VoxelGrid<PointT> downsampler;
	downsampler.setInputCloud(inputCloud);
	downsampler.setLeafSize(voxel_size, voxel_size, voxel_size);
	downsampler.filter(*cloud_filtered);
	return cloud_filtered;
}

PointCloudT::Ptr noiseFiltering(PointCloudT::Ptr inputCloud, int neighbors=5) {
	PointCloudT::Ptr cloud_filtered(new PointCloudT);

	pcl::RadiusOutlierRemoval<PointT> outrem;
	// build the filter
	outrem.setInputCloud(inputCloud);
	outrem.setRadiusSearch(0.08);
	outrem.setMinNeighborsInRadius(neighbors);
	// apply filter
	outrem.filter(*cloud_filtered);
	return cloud_filtered;
}

int main(int argc,	char* argv[])
{
	// The point clouds we will be using
	PointCloudT::Ptr cloud_in(new PointCloudT);  // Original point cloud
	PointCloudT::Ptr cloud_tr(new PointCloudT);  // Transformed point cloud
	PointCloudT::Ptr cloud_icp(new PointCloudT);  // ICP output point cloud

	pcl::console::TicToc time;
	// Read data from input.txt
	string cam_mat_path = "D:\\cammat.mat";
	string dist_mat_path = "D:\\distmat.mat";
	Mat cam_mat = matread(cam_mat_path);
	Mat dist_mat = matread(dist_mat_path);
	cout << "cam_mat" << cam_mat << endl;
	cout << "dist_mat" << dist_mat << endl;

	string ROOT_DIR = "D:\\data\\";
	ifstream index(string(ROOT_DIR + "input.txt").c_str());
	int i = 0;

	bool perform_icp = false;

	time.tic();
	while (!index.eof()) {
		string img_path;
		index >> img_path;
		img_path = ROOT_DIR + img_path;

		string color_path = img_path + "_rgb.png";
		string depth_path = img_path + "_depth.mat";
		string rot_path = img_path + "_rot.mat";
		string trans_path = img_path + "_trans.mat";

		Mat color, color_mapped, depth, rot, trans;

		color = imread(color_path);
		depth = matread(depth_path);
		rot = matread(rot_path);
		trans = matread(trans_path);

		cout << "i = " << i << endl;
		cout << "rot " << rot << endl;

		if (perform_icp) {
			if (i == 0) {
				convert_to_pointclouds(*cloud_in, color, depth, rot, trans);
			}
			else if (i == 1) {
				convert_to_pointclouds(*cloud_icp, color, depth, rot, trans);
				break;
			} else if (i > 1) {
				break;
			}
		}
		else {
			convert_to_pointclouds(*cloud_in, color, depth, rot, trans);
		}
		i++;
	}
	std::cout << "Loading data and convert to clouds in " << time.toc() << " ms" << std::endl;
	if (perform_icp) {
		// filter to remove the box
		PointCloudT::Ptr extracted_cloud(new PointCloudT());
		pcl::PassThrough<PointT> pass;
		pass.setInputCloud(cloud_in);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(-0.03, 1); // 0.1: 10cm
		pass.filter(*cloud_in);

		pass.setInputCloud(cloud_icp);
		pass.filter(*cloud_icp);
	}

	// Save to ROOT_DIR\model.pcd
	//time.tic();
	//pcl::io::savePCDFileASCII(ROOT_DIR + "model.pcd", *cloud_in);
	//std::cout << "Save model (" << cloud_in->points.size() << " points) to " << ROOT_DIR + "model.pcd in " << time.toc() << " ms" << std::endl;

	//time.tic();
	PointCloudT::Ptr cloud_down = downsampleCloud(cloud_in, 0.002);
	pcl::io::savePCDFileASCII(ROOT_DIR + "model_down.pcd", *cloud_down);
	std::cout << "Save down-sampled model (" << cloud_down->points.size() << " points) to " << ROOT_DIR + "model.pcd in " << time.toc() << " ms" << std::endl;

	//*cloud_in = *cloud_down; // visualize down sampled points

	if (perform_icp) {
		// Radius filtering
		// time.tic();
		//*cloud_in = *noiseFiltering(cloud_in);
		//*cloud_icp = *noiseFiltering(cloud_icp);
		//std::cout << "Applied noise filtering in " << time.toc() << " ms" << std::endl;

		*cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use

		 // The Iterative Closest Point algorithm
		time.tic();
		int iterations = 2000;  // Default number of ICP iterations

		PointCloudT::Ptr down_src = downsampleCloud(cloud_icp);
		PointCloudT::Ptr down_target = downsampleCloud(cloud_in);

		pcl::IterativeClosestPoint<PointT, PointT> icp;
		icp.setMaximumIterations(iterations); // for the next time we will call .align () function
		icp.setEuclideanFitnessEpsilon(1e-10);
		icp.setInputSource(down_src);
		icp.setInputTarget(down_target);
		icp.align(*down_src);

		std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;


		Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
		if (icp.hasConverged())
		{
			std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
			std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
			transformation_matrix = icp.getFinalTransformation().cast<double>();
			print4x4Matrix(transformation_matrix);

			pcl::transformPointCloud(*cloud_icp, *cloud_icp, icp.getFinalTransformation());
		}
		else
		{
			PCL_ERROR("\nICP has not converged.\n");
			return (-1);
		}
		
	}

	// Visualization
	pcl::visualization::PCLVisualizer viewer("ICP demo");
	// Create two verticaly separated viewports
	int v1(0);
	int v2(1);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

	// The color we will be using
	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;

	// Original point cloud is white
	if (perform_icp) {
		pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
			(int)255 * txt_gray_lvl);
		//viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.addPointCloud(cloud_in, "cloud_in_v1", v1);

		pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_tr, 20, 180, 20);
		//viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);
		viewer.addPointCloud(cloud_tr, "cloud_tr_v1", v1);

		viewer.addPointCloud(cloud_in, "cloud_in_v2", v2);
		viewer.addPointCloud(cloud_icp, "cloud_icp_v2", v2);
	}
	else {
		viewer.addPointCloud(cloud_in, "cloud_in_v1", v1);
	}

	// Set camera position and orientation
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1280, 1024);  // Visualiser window size

	// Register keyboard callback :
	viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);
	viewer.addCoordinateSystem(1.0, "cloud", 0);

	// Display the visualiser
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();

		//// The user pressed "space" :
		//if (next_iteration)
		//{
		//	// The Iterative Closest Point algorithm
		//	time.tic();
		//	icp.align(*down_src);
		//	std::cout << "Applied 1 ICP iteration in " << time.toc() << " ms" << std::endl;
		//	pcl::transformPointCloud(*cloud_icp, *cloud_icp, icp.getFinalTransformation());

		//	if (icp.hasConverged())
		//	{
		//		printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
		//		std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
		//		transformation_matrix *= icp.getFinalTransformation().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
		//		print4x4Matrix(transformation_matrix);  // Print the transformation between original pose and current pose

		//		ss.str("");
		//		ss << iterations;
		//		std::string iterations_cnt = "ICP iterations = " + ss.str();
		//		viewer.updateText(iterations_cnt, 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
		//		viewer.updatePointCloud(cloud_icp, "cloud_icp_v2");
		//	}
		//	else
		//	{
		//		PCL_ERROR("\nICP has not converged.\n");
		//		return (-1);
		//	}
		//}
		//next_iteration = false;
	}
	return (0);
}