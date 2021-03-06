#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

using namespace std;
using namespace cv;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

double FOCAL_X = 616.9788256385079;
double FOCAL_Y = 618.2612204459588;
double CENTER_X = 319.6980207838996;
double CENTER_Y = 237.2675723659526;
double MM_PER_M = 1000;
int WIDTH = 640;
int HEIGHT = 480;
double DEPTH_THRESHOLD = 1.2;   /*The largest depth we capture*/

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
			unsigned short depth = depth_img.at<unsigned short>(i, j);
			if (depth != 0 && depth / MM_PER_M < DEPTH_THRESHOLD) {
				Mat camPoint = Mat::zeros(3, 1, CV_64FC1);
				camPoint.at<double>(0, 0) = (j - CENTER_X) * depth / FOCAL_X / MM_PER_M;
				camPoint.at<double>(1, 0) = -(i - CENTER_Y) * depth / FOCAL_Y / MM_PER_M;
				camPoint.at<double>(2, 0) = double(depth) / MM_PER_M;

				// Mat worldPoint = rot_mat * camPoint - rot_mat.t() * trans_mat;
				// Mat worldPoint = rot_mat * camPoint + rot_mat.t() * trans_mat;
				// Mat worldPoint = camPoint + rot_mat.t() * trans_mat;
				Mat worldPoint = rot_mat * camPoint + trans_mat;

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


int
main(int argc,
	char* argv[])
{
	// The point clouds we will be using
	PointCloudT::Ptr cloud_in(new PointCloudT);  // Original point cloud
	PointCloudT::Ptr cloud_tr(new PointCloudT);  // Transformed point cloud
	PointCloudT::Ptr cloud_icp(new PointCloudT);  // ICP output point cloud

												  // Checking program arguments

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
		Mat color_u, depth_u;
		undistort(color, color_u, cam_mat, dist_mat);
		undistort(depth, depth_u, cam_mat, dist_mat);

		if (i == 0) {
			convert_to_pointclouds(*cloud_in, color_u, depth_u, rot, trans);
		}
		else if (i == 1) {
			convert_to_pointclouds(*cloud_icp, color_u, depth_u, rot, trans);
		}
		else if (i >= 1) {
			break;
		}
		i++;
	}

	*cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use

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
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
		(int)255 * txt_gray_lvl);
	viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
	viewer.addPointCloud(cloud_in, "cloud_in_v2", v2);

	// Transformed point cloud is green
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_tr, 20, 180, 20);
	viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);
	viewer.addPointCloud(cloud_icp, "cloud_icp_v2", v2);

	// Set background color
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

	// Set camera position and orientation
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1280, 1024);  // Visualiser window size

	// Display the visualiser
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	return (0);
}