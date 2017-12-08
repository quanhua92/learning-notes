#include <iostream>
#include <librealsense2\rs.hpp>
#include <pcl\visualization\pcl_visualizer.h>

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

float get_depth_scale(rs2::device dev);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);

void main(int argc, char** argv) {
	rs2::pointcloud pc;
	rs2::points points;

	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

	rs2::pipeline pipe;
	rs2::pipeline_profile profile = pipe.start(cfg);

	//float depth_scale = get_depth_scale(profile.get_device());

	rs2_stream align_to = find_stream_to_align(profile.get_streams());

	rs2::align align(align_to);

	pcl::visualization::PCLVisualizer *p = new pcl::visualization::PCLVisualizer(argc, argv, "3D");
	p->setSize(1080, 720);
	p->addCoordinateSystem(1.0);
		
	bool is_first = true;
	
	while (!p->wasStopped()) {
		rs2::frameset frameset = pipe.wait_for_frames();

		rs2::frameset data = align.proccess(frameset);

		rs2::depth_frame depth_frame = data.get_depth_frame();
		rs2::video_frame color_frame = data.first(align_to);

		points = pc.calculate(depth_frame);
		pc.map_to(color_frame);

		auto vertices = points.get_vertices();
		const rs2::texture_coordinate* tex_coords = points.get_texture_coordinates();

		PointCloudT::Ptr cloud(new PointCloudT);

		int width = color_frame.get_width();
		int height = color_frame.get_height();

		int d_width = depth_frame.get_width();
		int d_height = depth_frame.get_height();

		int cloud_size = points.size();

		uint8_t* p_color_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(color_frame.get_data()));
		int color_bpp = color_frame.get_bytes_per_pixel();

		#pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
		for (int y = 0; y < height; y++)
		{
			auto depth_pixel_index = y * width;
			for (int x = 0; x < width; x++, ++depth_pixel_index)
			{
				int i = (int)depth_pixel_index;
				if (vertices[i].z) {
				    PointT point;
					point.x = vertices[i].x;
					point.y = vertices[i].y;
					point.z = vertices[i].z;

					int offset = depth_pixel_index * color_bpp;
					point.r = p_color_frame[offset++];
					point.g = p_color_frame[offset++];
					point.b = p_color_frame[offset++];
				    cloud->points.push_back(point);
				}
			}
		}

		cloud->width = cloud->points.size();
		cloud->height = 1;
		if (is_first) {
			p->addPointCloud(cloud, "cloud");
			is_first = false;
		}
		else {
			p->updatePointCloud(cloud, "cloud");
		}
		p->spinOnce();
		//cout << "mean_depth: " << mean_depth / count << " count: " << count << " size: " << points.size() << "\r";
	}

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
