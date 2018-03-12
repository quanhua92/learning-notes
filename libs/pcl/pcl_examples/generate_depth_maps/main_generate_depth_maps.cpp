#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

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
bool do_visualize = false;

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

void createSegmentationMap(cv::Mat &depthImage, cv::Mat &horizontal, cv::Mat &vertical, int &left, int &top, int &bottom, int &right) {
	double min;
	double max;
	cv::minMaxIdx(depthImage, &min, &max);
	cv::Mat adjMap;
	cv::convertScaleAbs(depthImage, adjMap, 255 / max);

	vector<Point2i> locations;
	findNonZero(adjMap, locations);

	bottom = 0, right = 0, left = depthImage.cols, top = depthImage.rows - 1;

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

cv::Mat getDepthMap(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ> &sparse_cloud, cv::Mat &img_depth, cv::Mat &index_map, vector<Point2f> &source_points) {
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
	PointCloud<PointXYZ>::Ptr mean_cloud(new PointCloud<PointXYZ>());
	PointCloud<PointXYZ>::Ptr mean_sparse_cloud(new PointCloud<PointXYZ>());
	if (io::loadPCDFile("D:\\mhmodels_old\\mean_shape\\mean_shape_hd.pcd", *mean_cloud) < 0) {
		cout << "Error loading mean_cloud" << endl;
		showHelp(argv[0]);
		system("pause");
		return -1;
	}

	if (io::loadPCDFile("D:\\mhmodels_old\\mean_shape\\mean_shape.pcd", *mean_sparse_cloud) < 0) {
		cout << "Error loading mean_cloud" << endl;
		showHelp(argv[0]);
		system("pause");
		return -1;
	}
	cout << "mean_shape points: " << mean_cloud->points.size() << endl;


	// Translation
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << 0, 1.0, 0.1;
	pcl::transformPointCloud(*mean_cloud, *mean_cloud, transform);
	pcl::transformPointCloud(*mean_sparse_cloud, *mean_sparse_cloud, transform);

	Mat mean_depth_img_ori;
	Mat mean_depth_index_map_ori;
	vector<Point2f> mean_points;
	getDepthMap(*mean_cloud, *mean_sparse_cloud, mean_depth_img_ori, mean_depth_index_map_ori, mean_points);

	// DO SEGMENTATION
	Mat horizontal_ori, vertical_ori;
	int left, top, bottom, right;
	createSegmentationMap(mean_depth_img_ori, horizontal_ori, vertical_ori, left, top, bottom, right);
	Rect rect(left, top, right - left, bottom - top);
	Mat mean_depth_img = mean_depth_img_ori(rect);
	Mat mean_depth_index_map = mean_depth_index_map_ori(rect);
	Mat horizontal = horizontal_ori(rect);
	Mat vertical = vertical_ori(rect);
	for (int i = 0; i < mean_points.size(); i++) {
		mean_points[i] = Point2f(mean_points[i].x - left, mean_points[i].y - top);
	}

	{
		FileStorage fs;
		fs.open("D:\\mean_points.yml.gz", FileStorage::WRITE);
		fs.write("mean_points", mean_points);
		fs.write("mean_depth_index_map", mean_depth_index_map);
		fs.write("horizontal", horizontal);
		fs.write("vertical", vertical);
		fs.write("mean_depth_img", mean_depth_img);
		fs.release();
	}

	ifstream f(argv[1]);
	string model_path;
	while (std::getline(f, model_path)) {
		model_path = "D:\\mhmodels_old\\models\\00240";
		cout << "model_path: " << model_path << endl;

		string dense_path = model_path + "_hd.pcd";
		string sparse_path = model_path + ".pcd";

		PointCloud<PointXYZ>::Ptr source_cloud(new PointCloud<PointXYZ>());
		PointCloud<PointXYZ>::Ptr sparse_cloud(new PointCloud<PointXYZ>());
		if (io::loadPCDFile(dense_path, *source_cloud) < 0) {
			cout << "Error loading " << dense_path << endl;
			showHelp(argv[0]);
			system("pause");
			return -1;
		}

		if (io::loadPCDFile(sparse_path, *sparse_cloud) < 0) {
			cout << "Error loading " << sparse_path << endl;
			showHelp(argv[0]);
			system("pause");
			return -1;
		}

		pcl::transformPointCloud(*source_cloud, *source_cloud, transform);
		pcl::transformPointCloud(*sparse_cloud, *sparse_cloud, transform);

		// GET DEPTH MAP
		Mat output_depth_img;
		Mat output_index_map;
		vector<Point2f> source_points;
		Mat rgb_img = getDepthMap(*source_cloud, *sparse_cloud, output_depth_img, output_index_map, source_points);

		double min;
		double max;
		cv::minMaxIdx(output_depth_img, &min, &max);
		cv::Mat adjMap;
		cv::convertScaleAbs(output_depth_img, adjMap, 255 / max);
		//imwrite("D:\\depth_img.png", depth_img);

		// POINT SURFACE SPLINES

		vector<DMatch> matches;
		for (int landmark_index = 0; landmark_index < SELECTED_LANDMARKS.size(); landmark_index++) {
			matches.push_back(DMatch(landmark_index, landmark_index, 0));
		}
		cv::Ptr<cv::ThinPlateSplineShapeTransformer> tps;
		tps = cv::createThinPlateSplineShapeTransformer(0);
		tps->estimateTransformation(source_points, mean_points, matches);

		vector<Point2f> image_points;
		for (int x = 0; x < IMG_SIZE; x++) {
			for (int y = 0; y < IMG_SIZE; y++) {
				image_points.push_back(Point2f(x, y));
			}
		}
		Mat output_points;
		tps->applyTransformation(image_points, output_points);

		Mat outputMap = Mat::zeros(Size(IMG_SIZE, IMG_SIZE), CV_8UC3);

		Mat rgb_img_hor = rgb_img.clone();
		Mat rgb_img_ver = rgb_img.clone();

		Mat output_uv_space = Mat::zeros(Size(IMG_SIZE, IMG_SIZE), CV_32FC2);
		Mat output_seg = Mat::zeros(Size(IMG_SIZE, IMG_SIZE), CV_8UC2);

		{
			Eigen::Affine3f transform = Eigen::Affine3f::Identity();
			transform.translation() << 0, 1.0, 0.1;
			pcl::transformPointCloud(*source_cloud, *source_cloud, transform);
		}

		pcl::visualization::PCLVisualizer viewer("Extract plane example");
		float bckgr_gray_level = 0.0;  // Black
		viewer.addCoordinateSystem(1.0, "cloud", 0);
		viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);
		viewer.setCameraPosition(0.05, 0.05, 0.3, 0, 0, -1);
		viewer.setSize(1280, 1024);  // Visualiser window size

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> model_cloud_color_handler(mean_cloud, 155, 155, 155);
		viewer.addPointCloud(mean_cloud, model_cloud_color_handler, "mean_cloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "mean_cloud");

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scan_cloud_color_handler(source_cloud, 155, 155, 155);
		viewer.addPointCloud(source_cloud, scan_cloud_color_handler, "source_cloud");
		int line = 0;

		for (int x = 0; x < IMG_SIZE; x++) {
			for (int y = 0; y < IMG_SIZE; y++) {
				int depth_value = output_depth_img.at<unsigned short>(y, x);
				if (depth_value > 0) {
					int index = x * IMG_SIZE + y;
					Point2f new_location = output_points.at<Point2f>(0, index);
					if (new_location.x > 0 && new_location.y > 0 && new_location.x < IMG_SIZE && new_location.y < IMG_SIZE) {

							cout << "i: " << x << " j: " << y << endl;
							cout << "index: " << index << " x : " << new_location.x << " " << new_location.y << endl;


						int h_bin = horizontal.at<char>(int(new_location.y), int(new_location.x));
						int v_bin = vertical.at<char>(int(new_location.y), int(new_location.x));

						Vec2f uv_space;
						Vec2b seg_id;

						uv_space[0] = new_location.x * 1.0 / mean_depth_img.cols;
						uv_space[1] = new_location.y * 1.0 / mean_depth_img.rows;
						seg_id[0] = h_bin;
						seg_id[1] = v_bin;

						output_uv_space.at<Vec2f>(y, x) = uv_space;
						output_seg.at<Vec2b>(y, x) = seg_id;

						// FOR VISUALIZE
						if (do_visualize) {
							Vec3b original_color = rgb_img.at<Vec3b>(y, x);
							Vec3b color;
							color[0] = original_color[0];
							color[1] = original_color[1];
							color[2] = original_color[2];

							outputMap.at<Vec3b>(int(new_location.y), int(new_location.x)) = color;

							if (h_bin > 0) {
								rgb_img_hor.at<Vec3b>(y, x) = getBinColor(h_bin, NUM_OF_BINS);
							}

							if (v_bin > 0) {
								rgb_img_ver.at<Vec3b>(y, x) = getBinColor(v_bin, NUM_OF_BINS);
							}


						}
						int scan_index = int(output_index_map.at<float>(y, x));
						int model_index = int(mean_depth_index_map.at<float>(int(new_location.y), int(new_location.x)));

						if (scan_index > 0 && model_index > 0) {
							stringstream ss;
							ss << "line: " << line++;
							cout << "model_index: " << model_index << " scan_index: " << scan_index << endl;
							viewer.addLine(source_cloud->points[scan_index], mean_cloud->points[model_index], ss.str());

							//while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
							//	viewer.spinOnce();
							//}
						}


					}
				}
			}
		}

		string storage_path = model_path + ".yml.gz";
		FileStorage fs;
		fs.open(storage_path, FileStorage::WRITE);
		fs.write("depth_img", output_depth_img);
		fs.write("index_map", output_index_map);
		fs.write("uv_space", output_uv_space);
		fs.write("segmentation", output_seg);
		fs.release();
		cout << "Write: " << storage_path << endl;

		if (do_visualize) {
			cv::imshow("rgb_img", rgb_img);
			cv::imshow("output_depth_img", adjMap);
			cv::imshow("rgb_img_hor", rgb_img_hor);
			cv::imshow("rgb_img_ver", rgb_img_ver);
			cv::imshow("outputMap", outputMap);
			cv::waitKey(0);
		}


		// Visualization

		// DRAW CORRES
		//{
		//	int line = 0;
		//	for (int i = 0; i < output_index_map.cols; i++) {
		//		for (int j = 0; j < output_index_map.rows; j++) {
		//			unsigned short scan_index = output_index_map.at<unsigned short>(j, i);
		//			int index = i * IMG_SIZE + j;
		//			Point2f new_location = output_points.at<Point2f>(0, index);
		//			if (new_location.x > 0 && new_location.y > 0) {
		//				cout << "i: " << i << " j: " << j << endl;
		//				cout << "index: " << index << " x : " << new_location.x << " " << new_location.y << endl;
		//			}
		//			unsigned short model_index = mean_depth_index_map.at<unsigned short>(int(new_location.y), int(new_location.x));

		//			if (scan_index > 0 && model_index > 0) {
		//				stringstream ss;
		//				ss << "line: " << line++;
		//				viewer.addLine(source_cloud->points[scan_index], mean_cloud->points[model_index], ss.str());
		//			}
		//		}
		//	}
		//}


		//// Visualization
		
		//int v1(0);
		//int v2(1);
		//viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		//viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

		//float bckgr_gray_level = 0.0;  // Black

		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255, 255, 255);
		//viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original_cloud", v1);
		//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");

		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sparse_cloud_color_handler(sparse_cloud, 255, 255, 255);
		//viewer.addPointCloud(sparse_cloud, sparse_cloud_color_handler, "sparse_cloud", v2);
		//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sparse_cloud");

		//viewer.addCoordinateSystem(1.0, "cloud", 0);
		//viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
		//viewer.setCameraPosition(0.05, 0.05, 0.3, 0, 0, -1);

		//viewer.setSize(1280, 1024);  // Visualiser window size

		while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
			viewer.spinOnce();
		}

	} // end while getline()

	system("pause");

	return 0;
}
