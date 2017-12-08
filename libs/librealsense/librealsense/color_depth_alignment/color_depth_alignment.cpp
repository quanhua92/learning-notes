#include <iostream>
#include <librealsense2\rs.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
float get_depth_scale(rs2::device dev);
static void get_field_of_view(const rs2::stream_profile& stream);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);

int main() try {
	//rs2::config cfg;
	////cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 60);
	//cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);
	//cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);

	rs2::pipeline pipe;
	//rs2::pipeline_profile profile = pipe.start(cfg);
	rs2::pipeline_profile profile = pipe.start();

	float depth_scale = get_depth_scale(profile.get_device());

	rs2_stream align_to = find_stream_to_align(profile.get_streams());

	rs2::align align(align_to);

	//get_field_of_view(profile.get_stream(RS2_STREAM_COLOR));

	while (true) {
		rs2::frameset frameset = pipe.wait_for_frames();

		// wait_for_frames can replace the device in case of device error or disconnection.
		// we will need to use function profile_changed in
		// https://github.com/IntelRealSense/librealsense/blob/master/examples/align/rs-align.cpp#L60

		// get processed aligned frame
		rs2::frameset data = align.proccess(frameset);

		rs2::depth_frame depth_frame = data.get_depth_frame();
		rs2::video_frame color_frame = data.first(align_to);


		Mat color(Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

		Mat depth(Size(depth_frame.get_width(), depth_frame.get_height()), CV_16UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

		imshow("Color", color);
		imshow("Depth", depth);
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

float get_depth_scale(rs2::device dev)
{
	// Go over the device's sensors
	for (rs2::sensor& sensor : dev.query_sensors())
	{
		// Check if the sensor if a depth sensor
		if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
		{
			return dpt.get_depth_scale();
		}
	}
	throw std::runtime_error("Device does not have a depth sensor");
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
	//Given a vector of streams, we try to find a depth stream and another stream to align depth with.
	//We prioritize color streams to make the view look better.
	//If color is not available, we take another stream that (other than depth)
	rs2_stream align_to = RS2_STREAM_ANY;
	bool depth_stream_found = false;
	bool color_stream_found = false;
	for (rs2::stream_profile sp : streams)
	{
		rs2_stream profile_stream = sp.stream_type();
		if (profile_stream != RS2_STREAM_DEPTH)
		{
			if (!color_stream_found)         //Prefer color
				align_to = profile_stream;

			if (profile_stream == RS2_STREAM_COLOR)
			{
				color_stream_found = true;
			}
		}
		else
		{
			depth_stream_found = true;
		}
	}

	if (!depth_stream_found)
		throw std::runtime_error("No Depth stream available");

	if (align_to == RS2_STREAM_ANY)
		throw std::runtime_error("No stream found to align with Depth");

	return align_to;
}

static void get_field_of_view(const rs2::stream_profile& stream)
{
	// A sensor's stream (rs2::stream_profile) is in general a stream of data with no specific type.
	// For video streams (streams of images), the sensor that produces the data has a lens and thus has properties such
	//  as a focal point, distortion, and principal point.
	// To get these intrinsics parameters, we need to take a stream and first check if it is a video stream
	if (auto video_stream = stream.as<rs2::video_stream_profile>())
	{
		try
		{
			//If the stream is indeed a video stream, we can now simply call get_intrinsics()
			rs2_intrinsics intrinsics = video_stream.get_intrinsics();

			auto principal_point = std::make_pair(intrinsics.ppx, intrinsics.ppy);
			auto focal_length = std::make_pair(intrinsics.fx, intrinsics.fy);
			rs2_distortion model = intrinsics.model;

			std::cout << "Principal Point         : " << principal_point.first << ", " << principal_point.second << std::endl;
			std::cout << "Focal Length            : " << focal_length.first << ", " << focal_length.second << std::endl;
			std::cout << "Distortion Model        : " << model << std::endl;
			std::cout << "Distortion Coefficients : [" << intrinsics.coeffs[0] << "," << intrinsics.coeffs[1] << "," <<
				intrinsics.coeffs[2] << "," << intrinsics.coeffs[3] << "," << intrinsics.coeffs[4] << "]" << std::endl;
		}
		catch (const std::exception& e)
		{
			std::cerr << "Failed to get intrinsics for the given stream. " << e.what() << std::endl;
		}
	}
	else
	{
		std::cerr << "Given stream profile is not a video stream profile" << std::endl;
	}
}