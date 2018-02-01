#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;

int main(int argc, char** argv) {
	const Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04, 0.02, dictionary);
	cv::Mat boardImage;
	board->draw(cv::Size(600, 500), boardImage, 10, 1);
	imshow("board", boardImage);
	imwrite("D:\\charuco_board.png", boardImage);
	waitKey(0);

	return EXIT_SUCCESS;
}