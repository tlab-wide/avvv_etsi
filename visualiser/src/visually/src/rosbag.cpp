#include <visually/rosbag.hpp>

namespace rosbag
{

RosbagManipulator::RosbagManipulator()
	: reader_()
	, writer_()
{

}

RosbagManipulator::~RosbagManipulator()
{

}

void RosbagManipulator::getAllTopics(
	const std::string& filename
	, std::vector<std::string>& topic_names)
{
	auto file_path = ament_index_cpp::get_package_share_directory("visually")
		+ "/data/" + filename;
	reader_.open(file_path);

	for (const auto& topic : reader_.get_all_topics_and_types())
		topic_names.push_back(topic.name);
}

} // namespace rosbag