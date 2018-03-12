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
	10958, 10965, 10978, 10979, 11052, 11079, 11109, 11147, 11193, 11291, 11325, 11348, 11365, 11375, 11936 
};

void showHelp(char * program_name) {
	cout << endl;
	cout << "Usage: " << program_name << " D:\data\bodidata-quan\exportFinalCloud.pcd D:\scan_depth_map.png D:\scan_index_map.png D:\scan_depth_result.yml.gz D:\mhmodels_old\models\00007.pcd D:\mean_points.yml.gz" << endl;
	cout << "-h: Show this help." << endl;
}

void getUVSpace(pcl::PointCloud<pcl::PointXYZ> &sparse_cloud, vector<Point2f> &source_points) {
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
}

Vec3b getColorFromHue(float hue) {
	Mat rgb;
	Mat hsv(1, 1, CV_8UC3, Scalar(hue * 255, 255, 255));
	cvtColor(hsv, rgb, CV_HSV2BGR);
	Vec3b color;
	color[0] = rgb.data[0]; color[1] = rgb.data[1]; color[2] = rgb.data[2];

	return color;
}


int main(int argc, char* argv[]) {

	if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help")) {
		showHelp(argv[0]);
	}

	if (argc != 7) {
		showHelp(argv[0]);
		std::system("pause");
		return -1;
	}

	string scan_cloud_path = argv[1];
	string scan_depth_map_path = argv[2];
	string scan_depth_index_map_path = argv[3];
	string scan_depth_result_path = argv[4];
	string model_path = argv[5];
	string mean_points_path = argv[6];

	// load current model
	PointCloud<PointXYZ>::Ptr model_sparse_cloud(new PointCloud<PointXYZ>());
	if (io::loadPCDFile(model_path, *model_sparse_cloud) < 0) {
		cout << "Error loading model cloud" << endl;
		showHelp(argv[0]);
		std::system("pause");
		return -1;
	}

	// Translation
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << 0, 1.0, 0.1;
	transformPointCloud(*model_sparse_cloud, *model_sparse_cloud, transform);

	// Load mean_points
	Mat mean_depth_img;
	vector<Point2f> mean_points;
	{
		FileStorage fs;
		fs.open(mean_points_path, cv::FileStorage::READ);
		mean_depth_img = fs["mean_depth_img"].mat();
		FileNode fn = fs["mean_points"];
		Mat mean_points_mat = fn.mat();
		Point2f v;
		for (int i = 0; i < SELECTED_LANDMARKS.size(); i++) {
			v = mean_points_mat.at<Point2f>(0, i);
			mean_points.push_back(v);
		}
	}

	// Create UV_SPACE of the current model
	vector<Point2f> model_sparse_points;
	getUVSpace(*model_sparse_cloud, model_sparse_points);

	vector<Point2f> model_uv_space;
	{
		// calculate uv space 
		vector<DMatch> matches;
		for (int landmark_index = 0; landmark_index < SELECTED_LANDMARKS.size(); landmark_index++) {
			matches.push_back(DMatch(landmark_index, landmark_index, 0));
		}

		cv::Ptr<cv::ThinPlateSplineShapeTransformer> tps;
		tps = cv::createThinPlateSplineShapeTransformer(0);
		tps->estimateTransformation(model_sparse_points, mean_points, matches);
		Mat output_points;
		tps->applyTransformation(model_sparse_points, output_points);

		Point2f v;
		for (int i = 0; i < SELECTED_LANDMARKS.size(); i++) {
			v = output_points.at<Point2f>(0, i);
			model_uv_space.push_back(v);
		}
	}

	// load scan model
	PointCloud<PointXYZ>::Ptr scan_cloud(new PointCloud<PointXYZ>());
	if (io::loadPCDFile(scan_cloud_path, *scan_cloud) < 0) {
		cout << "Error loading scan cloud" << endl;
		showHelp(argv[0]);
		std::system("pause");
		return -1;
	}
	{
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		// rotate scan -> mean
		transform.rotate(Eigen::AngleAxisf(+M_PI / 2, Eigen::Vector3f::UnitY()));
		transformPointCloud(*scan_cloud, *scan_cloud, transform);
		transform.setIdentity();
		transform.translation() << -0.5, 0, 0.5;
		transformPointCloud(*scan_cloud, *scan_cloud, transform);
		transform.rotate(Eigen::AngleAxisf(+M_PI, Eigen::Vector3f::UnitZ()));
		transformPointCloud(*scan_cloud, *scan_cloud, transform);
	}

	Mat scan_depth_map = imread(scan_depth_map_path, cv::IMREAD_UNCHANGED);
	Mat scan_depth_index_map = imread(scan_depth_index_map_path, cv::IMREAD_UNCHANGED);
	Mat u_space, v_space;

	Mat ground_uv_space;
	{
		//FileStorage fs;
		//fs.open("D:\\mhmodels_old\\models\\00240.yml.gz", FileStorage::READ);
		//fs["uv_space"] >> ground_uv_space;
		//fs["index_map"] >> scan_depth_index_map;
		//fs["depth_img"] >> scan_depth_map;
	}
	{
		FileStorage fs;
		fs.open("D:\\scan_depth_index.yml.gz", FileStorage::READ);
		fs["scan_index_map"] >> scan_depth_index_map;
		fs["scan_depth_map"] >> scan_depth_map;
	}

	{
		FileStorage fs;
		fs.open(scan_depth_result_path, FileStorage::READ);
		fs["u_space"] >> u_space;
		fs["v_space"] >> v_space;

		//vector<Mat> spaces;
		//split(ground_uv_space, spaces);
		//u_space = spaces[0].clone();
		//v_space = spaces[1].clone();
	}
	cout << u_space.size() << " " << v_space.size() << endl;
	{
		Mat rgb_u = Mat::zeros(scan_depth_index_map.size(), CV_8UC3);
		Mat rgb_v = Mat::zeros(scan_depth_index_map.size(), CV_8UC3);

		for (int i = 0; i < scan_depth_index_map.cols; i++) {
			for (int j = 0; j < scan_depth_index_map.rows; j++) {
				int index = int(scan_depth_index_map.at<float>(j, i));

				if (index > 0) {
					float u = u_space.at<float>(j, i);
					float v = v_space.at<float>(j, i);

					rgb_u.at<Vec3b>(j, i) = getColorFromHue(u);
					rgb_v.at<Vec3b>(j, i) = getColorFromHue(v);
				}
			}
		}

		resize(rgb_u, rgb_u, Size(512, 512));
		resize(rgb_v, rgb_v, Size(512, 512));

		imshow("rgb_u", rgb_u);
		imshow("rgb_v", rgb_v);
		//waitKey(0);
	}

	Mat scan_corres = Mat::zeros(scan_depth_index_map.size(), CV_16UC1);

	{
		for (int i = 0; i < scan_depth_index_map.cols; i++) {
			for (int j = 0; j < scan_depth_index_map.rows; j++) {
				int index = int(scan_depth_index_map.at<float>(j, i));

				if (index > 0) {
					float u = u_space.at<float>(j, i);
					float v = v_space.at<float>(j, i);

					int match = -1;
					float match_dist = 0;
					float threshold_dist = 0.005;
					for (int h = 0; h < model_uv_space.size(); h++) {
						float m_u = model_uv_space[h].x * 1.0 / mean_depth_img.cols;
						float m_v = model_uv_space[h].y * 1.0 / mean_depth_img.rows;

						float dist = sqrt((u - m_u) * (u - m_u) + (v - m_v) * (v - m_v));
						if (dist < threshold_dist) {
							if (match == -1 || (match_dist > dist)) {
								match_dist = dist;
								match = h;
							}
						}
					}

					if (match > 0) {
						cout << "match: " << match << " " << match_dist << endl;
						scan_corres.at<unsigned short>(j, i) = SELECTED_LANDMARKS[match];
					}
				}

			}
		}
	}


	// Visualization
	pcl::visualization::PCLVisualizer viewer("Extract plane example");
	float bckgr_gray_level = 0.0;  // Black
	viewer.addCoordinateSystem(1.0, "cloud", 0);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);
	viewer.setCameraPosition(0.05, 0.05, 0.3, 0, 0, -1);
	viewer.setSize(1280, 1024);  // Visualiser window size

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> model_cloud_color_handler(model_sparse_cloud, 155, 155, 155);
	viewer.addPointCloud(model_sparse_cloud, model_cloud_color_handler, "model_sparse_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "model_sparse_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scan_cloud_color_handler(scan_cloud, 155, 155, 155);
	viewer.addPointCloud(scan_cloud, scan_cloud_color_handler, "scan_cloud");

	// DRAW CORRES
	{
		int line = 0;
		for (int i = 0; i < scan_depth_index_map.cols; i++) {
			for (int j = 0; j < scan_depth_index_map.rows; j++) {
				int scan_index = int(scan_depth_index_map.at<float>(j, i));
				unsigned short model_index = scan_corres.at<unsigned short>(j, i);

				if (scan_index > 0 && model_index > 0) {
					stringstream ss;
					ss << "line: " << line++;
					viewer.addLine(scan_cloud->points[scan_index], model_sparse_cloud->points[model_index], ss.str());
				}
			}
		}
	}


	while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce();
	}

	return EXIT_SUCCESS;
}