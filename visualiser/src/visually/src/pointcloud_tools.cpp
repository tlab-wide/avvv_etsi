#include <visually/pointcloud_tools.hpp>

namespace pointcloud_tools
{

PointcloudTools::PointcloudTools(
    rclcpp::Node& node
    , const std::string& base_frame)
    : node_(node)
    , base_frame_(base_frame)
    , publisher_(node_.create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud2", rclcpp::QoS(10)))
{
    
}

PointcloudTools::~PointcloudTools()
{
    
}

void PointcloudTools::loadFile(
    const std::string& file_path
    , const double offset[3]
    , bool centre)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, cloud) == -1)
        return;

    if (centre)
        centreCloud(cloud);
    
    if (offset[0] != 0 or offset[1] != 0 or offset[2] != 0)
        applyOffset(cloud, offset);

    waitForSubscriber();

    sensor_msgs::msg::PointCloud2 pc2_msg;

    createPointcloud2Msg(cloud, pc2_msg);

    publisher_->publish(pc2_msg);
}

void PointcloudTools::waitForSubscriber()
{
    using namespace std::chrono_literals;
    while (node_.count_subscribers("/pointcloud2") == 0)
        rclcpp::sleep_for(200ms);
}

void PointcloudTools::centreCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    // Calculating the averages of x, y and z coordinates
    long long count{ cloud.width * cloud.height };
    double average_x{ 0.0 };
    double average_y{ 0.0 };
    double average_z{ 0.0 };

    for (auto& point : cloud.points) {
        average_x += point.x;
        average_y += point.y;
        average_z += point.z;
    }

    average_x /= count;
    average_y /= count;
    average_z /= count;

    // Shifting each point as much as its fields average back to the origin
    for (auto& point : cloud.points) {
        point.x -= average_x;
        point.y -= average_y;
        point.z -= average_z;
    }
}

void PointcloudTools::applyOffset(pcl::PointCloud<pcl::PointXYZ>& cloud, const double offset[3])
{
    for (auto& point : cloud.points) {
        point.x += offset[0];
        point.y += offset[1];
        point.z += offset[2];
    }
}

void PointcloudTools::copyValueToData(float value, std::vector<unsigned char>& data)
{
    unsigned char bytes[4]{};
    std::copy(static_cast<const unsigned char*>(static_cast<const void*>(&value)),
                static_cast<const unsigned char*>(static_cast<const void*>(&value)) + sizeof(value),
                bytes);
    data.push_back(bytes[0]);
    data.push_back(bytes[1]);
    data.push_back(bytes[2]);
    data.push_back(bytes[3]);
}

void PointcloudTools::createPointcloud2Msg(
    const pcl::PointCloud<pcl::PointXYZ>& cloud
    , sensor_msgs::msg::PointCloud2& pc2_msg)
{
    pc2_msg.header.frame_id = base_frame_;
    pc2_msg.header.stamp = node_.now();

    pc2_msg.width = cloud.width;
    pc2_msg.height = cloud.height;

    // Building and configuring the fields of the message
    sensor_msgs::msg::PointField fields[]{
        sensor_msgs::msg::PointField()
        , sensor_msgs::msg::PointField()
        , sensor_msgs::msg::PointField()
    };

    std::string field_names{"xyz"};
    for (int i{ 0 }; i < 3; ++i) {
        fields[i].name = field_names[i];
        fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        fields[i].count = 1;
        fields[i].offset = 4 * i;
        pc2_msg.fields.push_back(fields[i]);
    }

    // The data is little endian
    pc2_msg.is_bigendian = false;

    pc2_msg.point_step = 4 * 3;
    pc2_msg.row_step = cloud.points.size() * pc2_msg.point_step;
    
    // Converting point float values to bytes and appending them to the message's data
    for (auto& point : cloud.points) {
            copyValueToData(point.x, pc2_msg.data);
            copyValueToData(point.y, pc2_msg.data);
            copyValueToData(point.z, pc2_msg.data);
    }

    pc2_msg.is_dense = false;
}

} // namespace pointcloud_tools
