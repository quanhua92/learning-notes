#include <iostream>
#include <librealsense2\rs.hpp>

void main() {
	rs2::pipeline pipe;

	pipe.start();

	while (true) {
		rs2::frameset frames = pipe.wait_for_frames();

		rs2::depth_frame depth = frames.get_depth_frame();

		float width = depth.get_width();
		float height = depth.get_height();

		float dist_to_center = depth.get_distance(width / 2, height / 2);

		std::cout << "The camera is facing an object " << dist_to_center << " meters away \r";
	}
}