#include <iostream>
#include <librealsense2\rs.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main() try {
	rs2::colorizer color_map;

	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);

	rs2::pipeline pipe;
	pipe.start(cfg);

	while (true) {
		rs2::frameset data = pipe.wait_for_frames();
		
		rs2::frame depth_frame = data.get_depth_frame();
		rs2::frame color_frame = data.get_color_frame();
		rs2::frame ir_frame    = data.first(RS2_STREAM_INFRARED);

		Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
		Mat depth(Size(640, 480), CV_16UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
		Mat ir(Size(640, 480), CV_8UC1, (void*)ir_frame.get_data(), Mat::AUTO_STEP);

		cout << "depth " << depth.at<unsigned short>(240, 320) << "\r";

		equalizeHist(ir, ir);
		applyColorMap(ir, ir, COLORMAP_JET);

		imshow("Color", color);
		imshow("Depth", depth);
		imshow("Infrared", ir);
		waitKey(30);
	}
	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}