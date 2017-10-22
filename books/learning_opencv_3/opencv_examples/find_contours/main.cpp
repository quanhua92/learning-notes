//#include <iostream>
//#include <opencv2/opencv.hpp>
//using namespace cv;
//using namespace std;
//
//int main(int argc, char** argv) {
//	cout << "path " << argv[1] << endl;
//	Mat image = imread(argv[1], IMREAD_GRAYSCALE);
//
//	vector<vector<cv::Point>> contours;
//	findContours(
//		image,
//		contours,
//		cv::noArray(),
//		cv::RETR_TREE,
//		cv::CHAIN_APPROX_SIMPLE
//	);
//	cv::imshow("Image", image);
//
//	image = cv::Scalar::all(0);
//
//	drawContours(image, contours, -1, cv::Scalar::all(255));
//	cv::imshow("contours", image);
//
//	// Find the convex hull object for each contour
//	vector<vector<Point> >hull(contours.size());
//	for (int i = 0; i < contours[1].size(); i++){
//		contours[0].push_back(contours[1][i]);
//	}
//	for (int i = 0; i < contours.size(); i++)
//	{
//		convexHull(Mat(contours[i]), hull[i], false);
//	}
//
//	RNG rng;
//	Mat drawing = Mat::zeros(image.size(), CV_8UC3);
//	drawContours(drawing, hull, 0, cv::Scalar::all(255));
//	for (int i = 2; i< contours.size(); i++)
//	{
//		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
//		//drawContours(drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point());
//		drawContours(drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point());
//	}
//
//	// Show in a window
//	namedWindow("Hull demo", CV_WINDOW_AUTOSIZE);
//	imshow("Hull demo", drawing);
//
//	waitKey(0);
//}