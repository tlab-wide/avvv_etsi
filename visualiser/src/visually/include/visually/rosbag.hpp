/**
 * @author AmirInt <amir.intelli@gmail.com>
 * @brief Provides tools to read rosbag file data from .db3 files
*/

#ifndef VISUALLY__ROSBAG_HPP_
#define VISUALLY__ROSBAG_HPP_

// C++
#include <string>
#include <vector>

// CMake
#include <ament_index_cpp/get_package_share_directory.hpp>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>


namespace rosbag
{

/**
 * @brief Handles rosbag files
*/
class RosbagManipulator
{
public:
    /**
     * @brief Constructor
    */
	RosbagManipulator();
	
    /**
     * @brief Deconstructor
    */
    ~RosbagManipulator();

    /**
     * @brief Reads and retrieves all messages of a specific given topic from
     * a given rosbag file
     * @throws "std::runtime_error" if there is an issue with opening the file
     * @param topic
     * @param filename
     * @param extracted_messages List of the obtained messages
    */
    template<typename MessageT>
    void readAllMessagesOfTopic(
        const std::string& topic
        , const std::string& filename
        , std::vector<MessageT>& extracted_messages)
    {
        auto file_path = ament_index_cpp::get_package_share_directory("visually")
            + "/data/" + filename;
        reader_.open(file_path);

        while (reader_.has_next()) {
            auto bag_message = reader_.read_next();
            if (bag_message->topic_name == topic) {
                MessageT msg;
                rclcpp::SerializedMessage extracted_serialised_message{ *bag_message->serialized_data };
                rclcpp::Serialization<MessageT> serialisation;
                serialisation.deserialize_message(&extracted_serialised_message, &msg);
                extracted_messages.push_back(msg);
            }
        }

        reader_.close();
    }

    /**
     * @brief Reads and retrieves all messages from a given rosbag file
     * @throws "std::runtime_error" if there is an issue with opening the file
     * @param filename
     * @param extracted_messages List of the obtained topic-message pairs
    */
    template<typename MessageT>
    void readAllMessages(
        const std::string& filename
        , std::vector<std::pair<std::string, MessageT>>& extracted_messages)
    {
        auto file_path = ament_index_cpp::get_package_share_directory("visually")
            + "/data/" + filename;
        reader_.open(file_path);

        while (reader_.has_next()) {
            auto bag_message = reader_.read_next();
            MessageT msg;
            rclcpp::SerializedMessage extracted_serialised_message{ *bag_message->serialized_data };
            rclcpp::Serialization<MessageT> serialisation;
            serialisation.deserialize_message(&extracted_serialised_message, &msg);
            extracted_messages.emplace_back(bag_message->topic_name, msg);
        }

        reader_.close();
    }

    /**
     * @brief Writes the given list of topic-message pairs into a given rosbag file
     * @throws "std::runtime_error" if there is an issue with opening the file
     * @param messages list of topic-message pairs
     * @param destination_filename
    */
    template<typename MessageT>
    void writeMessagesToBag(
        const std::vector<std::pair<std::string, MessageT>>& messages
        , const std::string& destination_filename)
    {
        auto file_path = ament_index_cpp::get_package_share_directory("visually")
            + "/data/" + destination_filename;
        writer_.open(file_path);

        std::string topic;

        for (const auto& message : messages)
            writer_.write<MessageT>(message.second, message.first, message.second.header.stamp);
    }

    /**
     * @brief Reads and retrieves all topics from a given rosbag file
     * @throws "std::runtime_error" if there is an issue with opening the file
     * @param filename
     * @param topic_names List of the obtained topics
    */
    void getAllTopics(
        const std::string& filename
        , std::vector<std::string>& topic_names);

private:
	rosbag2_cpp::Reader reader_;
    rosbag2_cpp::Writer writer_;
};

} // namespace rosbag

#endif // VISUALLY__ROSBAG_HPP_