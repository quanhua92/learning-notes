#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/common/transforms.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkPolyDataMapper.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <opencv2/opencv.hpp>
#include <pcl/console/time.h>   // TicToc
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <vtkInformation.h>
#include <vtkImageShiftScale.h>
#include <vtkPNGWriter.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <thread>
using namespace cv;
using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void showHelp(char * program_name) {
	cout << endl;
	cout << "Usage: " << program_name << " filename.txt <LOAD_IN_LOW_RES> <DO_VISUALIZATION>" << endl;
	cout << "LOAD_IN_LOW_RES: 1 -> use low res. 0 -> use high res" << endl;
	cout << "DO_VISUALIZATION: 1 -> display PCL visualizer. 0 -> do not display" << endl;
	cout << "Example: " << program_name << " filename.txt 1" << endl;
	cout << "-h: Show this help." << endl;
}

/** \brief Convert a \ref RangeImagePlanar object to a OpenCV Mat
* \param[in] outputImage: a OpenCV Mat which will be initialized with type CV_16UC1. each depth value is in milimeter.
* \param[in] rangeImage: the object that contains the data
*/
void convertRangeImagetoMat(cv::Mat &outputImage, const pcl::RangeImagePlanar& rangeImage)
{
	float MM_PER_M = 1000;
	outputImage = Mat::zeros(Size(rangeImage.width, rangeImage.height), CV_16UC1);

	for (int y = 0; y < rangeImage.height; y++)
	{
		for (int x = 0; x < rangeImage.width; x++)
		{
			outputImage.at<unsigned short>(y, x) = rangeImage(x, y).range * MM_PER_M;
		}
	}
}

void loadOBJinHighRes(string fileName, PointCloud<PointXYZ>::Ptr source_cloud, int detailLevel = 1) {
	// load in high res
	vtkSmartPointer<vtkPolyData> polydata1;
	vtkSmartPointer<vtkOBJReader> readerQuery = vtkSmartPointer<vtkOBJReader>::New();
	readerQuery->SetFileName(fileName.c_str());
	readerQuery->Update();
	polydata1 = readerQuery->GetOutput();
	//// polydata1->Update(); // this is not available now
	visualization::PCLVisualizer viewer;
	viewer.setShowFPS(false);
	viewer.addModelFromPolyData(polydata1, "mesh", 0);
	viewer.setRepresentationToSurfaceForAllActors();

	viewer.setCameraPosition(0.0, 1.0, 5, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

	PointCloud<PointXYZ>::Ptr frontal_cloud(new PointCloud<PointXYZ>());
	viewer.renderView(512, 512, frontal_cloud);

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << 0, 1, -5;
	transformPointCloud(*frontal_cloud, *frontal_cloud, transform);
	transform.setIdentity();
	transform.rotate(Eigen::AngleAxisf(+M_PI, Eigen::Vector3f::UnitY()));
	transformPointCloud(*frontal_cloud, *frontal_cloud, transform);

	PointCloud<PointXYZ>::Ptr sparse_cloud(new PointCloud<PointXYZ>());
	pcl::PCLPointCloud2 obj_cloud_ori;
	io::loadOBJFile(fileName, obj_cloud_ori);
	pcl::fromPCLPointCloud2(obj_cloud_ori, *sparse_cloud);

	transform.setIdentity();
	transform.rotate(Eigen::AngleAxisf(+M_PI/2, Eigen::Vector3f::UnitZ()));
	transformPointCloud(*frontal_cloud, *frontal_cloud, transform);
	transformPointCloud(*sparse_cloud, *sparse_cloud, transform);
	transform.setIdentity();
	transform.rotate(Eigen::AngleAxisf(+M_PI/2, Eigen::Vector3f::UnitY()));
	transformPointCloud(*frontal_cloud, *frontal_cloud, transform);
	transformPointCloud(*sparse_cloud, *sparse_cloud, transform);
	transform.setIdentity();

	viewer.removePolygonMesh("mesh");
	viewer.removePointCloud("mesh");
	viewer.removeShape("mesh");
	viewer.addCoordinateSystem(1.0);
	viewer.addPointCloud(sparse_cloud, "sparse_cloud");
	viewer.addPointCloud(frontal_cloud, "frontal_cloud");

	// We now want to create a range image from the above point cloud, with a 1deg angular resolution
	float angularResolution = (float)(0.1f * (M_PI / 180.0f));  //   1.0 degree in radians
	float maxAngleWidth = (float)(180.0f * (M_PI / 180.0f));  // 360.0 degree in radians
	float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 180.0 degree in radians

	viewer.setCameraPosition(3.0, 0.0, 1, 0.0, 0.0, 1.0, 0.0, 0.0, -1.0);
	Eigen::Affine3f sensorPose = viewer.getViewerPose();
	cout << sensorPose.matrix() << endl;

	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noiseLevel = 0.00;
	float minRange = 0.0f;
	int borderSize = 1;

	int width = 350;
	int height = 512;
	float fx = 612.197;
	float cx = width / 2;
	float fy = 616.048;
	float cy = height / 2;

	pcl::RangeImagePlanar rangeImage;
	rangeImage.createFromPointCloudWithFixedSize(*frontal_cloud, width, height,
		cx, cy, fx, fy,
		sensorPose, pcl::RangeImage::CAMERA_FRAME,
		noiseLevel, minRange);
	cout << rangeImage.width << " " << rangeImage.height << endl;
	Mat depthImg;
	convertRangeImagetoMat(depthImg, rangeImage);
/*
	pcl::RangeImage rangeImage;
	rangeImage.createFromPointCloud(*frontal_cloud, angularResolution, maxAngleWidth, maxAngleHeight,
		sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);*/
	pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
	range_image_widget.showRangeImage(rangeImage);

	std::cout << rangeImage << "\n";

	while (!viewer.wasStopped())
	{
		range_image_widget.spinOnce();
		viewer.spinOnce();
		std::this_thread::sleep_for(std::chrono::milliseconds(30));

		if (true)
		{
			sensorPose = viewer.getViewerPose();
			cout << sensorPose.matrix() << endl;
			rangeImage.createFromPointCloudWithFixedSize(*frontal_cloud, width, height,
				cx, cy, fx, fy,
				sensorPose, pcl::RangeImage::CAMERA_FRAME,
				noiseLevel, minRange);

			//rangeImage.createFromPointCloud(*frontal_cloud, angularResolution, maxAngleWidth, maxAngleHeight,
			//	sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);;
			range_image_widget.showRangeImage(rangeImage);
		}
	}

	while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce();
	}

	//int tesselated_sphere_level = detailLevel; // level 1: 50s
	//int resolution = 500;

	//PointCloud<PointXYZ>::CloudVectorType views_xyz;
	//std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
	//std::vector<float> enthropies;
	//viewer.renderViewTesselatedSphere(resolution, resolution, views_xyz, poses, enthropies, tesselated_sphere_level);
	//viewer.close();

	////take views and fuse them together
	//std::vector<PointCloud<PointXYZ>::Ptr> aligned_clouds;

	//for (size_t i = 0; i < views_xyz.size(); i++)
	//{
	//	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
	//	Eigen::Matrix4f pose_inverse;
	//	pose_inverse = poses[i].inverse();
	//	transformPointCloud(views_xyz[i], *cloud, pose_inverse);
	//	aligned_clouds.push_back(cloud);
	//}
	//for (size_t i = 0; i < aligned_clouds.size(); i++)
	//	*source_cloud += *aligned_clouds[i];
}

void processOBJFile(string LINE, bool useLowRes, bool doVisualization, int detailLevel = 1, int BOARD_HEIGHT = 800, int BOARD_WIDTH = 1200) {
	PointCloud<PointXYZ>::Ptr source_cloud(new PointCloud<PointXYZ>());

	string obj_file_path = LINE + ".obj";
	string pcd_file_path = LINE + ".pcd";

	if (useLowRes == false) {
		pcd_file_path = LINE + "_hd.pcd";
	}

	// check if file exists
/*	ifstream testFile(pcd_file_path);
	if (testFile.good()) {
		cout << "File " << pcd_file_path << " exists" << endl;
		return;
	}*/

	pcl::console::TicToc time;

	if (useLowRes) {
		pcl::PCLPointCloud2 obj_cloud_ori;
		time.tic();
		io::loadOBJFile(obj_file_path, obj_cloud_ori);
		pcl::fromPCLPointCloud2(obj_cloud_ori, *source_cloud);
		cout << "Load model (low res) in " << time.toc() << endl;
	}
	else {
		// load high res
		time.tic();
		loadOBJinHighRes(obj_file_path, source_cloud, detailLevel);
		cout << "Load model (high res) in " << time.toc() << endl;
	}

	// rotate to fix issue that MakeHuman model height in y axis
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.rotate(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitX()));
	PointCloud<PointXYZ>::Ptr final_cloud(new PointCloud<PointXYZ>());
	pcl::transformPointCloud(*source_cloud, *final_cloud, transform);
	transform.setIdentity();
	transform.rotate(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud(*final_cloud, *final_cloud, transform);

	bool debugSlice = false;
	if (debugSlice == false) {
		// save pcd file
		time.tic();
		pcl::io::savePCDFile(pcd_file_path, *final_cloud);
		cout << "Save model to pcd in " << time.toc() << " at " << pcd_file_path << endl;
	}
	else {
		cout << "Debug Slice Mode: Don't save model to pcd" << endl;
	}

	if (doVisualization) {
		// Visualization
		pcl::visualization::PCLVisualizer viewer("Batch Convert OBJ to PCD");

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 120, 120, 120);
		viewer.addPointCloud(source_cloud, source_cloud_color_handler, "source_cloud");

		viewer.addText("White: Source Cloud. Green: Final Cloud", 100, 100, "text");
		if (debugSlice == false) {
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_cloud_color_handler(final_cloud, 0, 120, 0);
			viewer.addPointCloud(final_cloud, final_cloud_color_handler, "final_cloud");
		}
		else {
			// debug: code to visualize a slice through model
			PointCloud<PointXYZ>::Ptr extracted_cloud(new PointCloud<PointXYZ>());
			pcl::PassThrough<PointXYZ> pass;
			pass.setInputCloud(final_cloud);
			pass.setFilterFieldName("z");
			pass.setFilterLimits(1., 1.01); //  0.1 = 10cm
			pass.filter(*extracted_cloud);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_cloud_color_handler(final_cloud, 0, 120, 0);
			viewer.addPointCloud(extracted_cloud, final_cloud_color_handler, "final_cloud");
		}

		viewer.addCoordinateSystem(1.0, "cloud", 0);
		viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

		while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
			viewer.spinOnce();
		}
	}
}

int main(int argc, char** argv) {
	if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help")) {
		showHelp(argv[0]);
	}

	if (argc != 4) {
		showHelp(argv[0]);
		system("pause");
		return -1;
	}

	bool useLowRes = atoi(argv[2]);
	bool doVisualization = atoi(argv[3]);

	if (useLowRes) {
		cout << "Loading in low resolution" << endl;
	}
	else {
		cout << "Loading in high resolution" << endl;
	}

	string INPUT_FILE_PATH = argv[1];
	int detailLevel = 1;
	ifstream file(INPUT_FILE_PATH.c_str());
	vector<string> listFiles;
	while (!file.eof()) {
		string line;
		file >> line;
		if (line.empty()) continue;
		listFiles.push_back(line);
	}

	for (int i = 0; i < listFiles.size(); i++) {
		string line = listFiles[i];
		cout << "Processing " << line << endl;
		pcl::console::TicToc time;
		time.tic();
		processOBJFile(line, useLowRes, doVisualization, detailLevel);
		double totalTime = time.toc();
		cout << "Done in " << totalTime << endl;

		double remainTime = totalTime * (listFiles.size() - i - 1) / 1000 / 60;
		cout << "Remain " << remainTime << " minutes" << endl;
	}

	system("pause");
	return 0;
}