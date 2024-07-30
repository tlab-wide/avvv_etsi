#include <visually/entities.hpp>

namespace entities
{

EchoPoints::EchoPoints()
{
    Eigen::Vector3d rotated;
    double angle;
    constexpr double angle_inc{ M_PI * 2 / (echo_line_points - 1) };
    
    geometry_msgs::msg::Point point;
    std::size_t i, j;

    for (i = 0; i < max_echo_line_radius * 2; ++i) {
        Eigen::Vector3d base((i + 1) * 0.5, 0.0, 0.0);
        for (j = 0, angle = 0.0; j < echo_line_points; ++j, angle += angle_inc) {
            rotated = Eigen::Matrix3d(
                Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ())) * base;
            point.x = rotated.x();
            point.y = rotated.y();
            point.z = rotated.z();
            points[i].push_back(point);
        }
    }
}

EchoPoints::~EchoPoints()
{

}

Entity::Entity(
    const std::shared_ptr<rclcpp::Node>& node
    , const std::string& id
    , const std::string& base_frame)
    : id_(id)
    , rvizer_(rviz_visual_tools::RvizVisualTools(base_frame, id, node))
{

}

Entity::Entity(const Entity& other)
    : id_(other.id_)
    , rvizer_(other.rvizer_)
{
    
}

Entity::~Entity()
{

}

std::string Entity::getId()
{
    return id_;
}

Mesh::Mesh(
    const std::shared_ptr<rclcpp::Node>& node
    , const std::string& base_frame
    , EntityType entity_type
    , const std::string& id
    , const std::string& colour
    , double scale
    , double opacity
    , int moving_line_layer_update_rate
    , const std::string& echo_colour)
    : Entity(node, id, base_frame)
    , entity_type_(entity_type)
    , colour_(transforms::getRvizColour(colour))
    , echo_colour_(transforms::getRvizColour(echo_colour))
    , scale_(scale)
    , opacity_(opacity)
    , display_info_(false)
    , info_(id)
    , moving_line_layer_(0)
    , moving_line_layer_update_rate_(moving_line_layer_update_rate)
{
    initialise(base_frame);
    text_scale_.x = text_scale_.y = text_scale_.z = 2.0;
    buildInteractiveMarker(base_frame);
    rvizer_.setAlpha(opacity_);
}

Mesh::~Mesh()
{
    rvizer_.deleteAllMarkers();
}

void Mesh::setPose(
    const geometry_msgs::msg::Pose& pose
    , const double position[3]
    , const double orientation[3])
{
    /**
     * @todo Use Eigen for transformations
    */
    pose_ = pose;

    double trfmx[4][4];
    transforms::getTransformMatrix(trfmx, position, orientation);
    double receiver_point[3];
    transforms::transformPoint(receiver_point, relative_receiver_position_, trfmx);
    receiver_position_.x = receiver_point[0];
    receiver_position_.y = receiver_point[1];
    receiver_position_.z = receiver_point[2];

    text_pose_.position = receiver_position_;
    text_pose_.position.z += 2;
    
    int_marker_.pose = pose;
}

void Mesh::setPose(
    const geometry_msgs::msg::Pose& pose)
{
    // Convert pose to straightforward arrays
    double position[3]{};
    double orientation[3]{};
    transforms::pose2Array(pose, position, orientation);
    setPose(pose, position, orientation);
}

void Mesh::toggleDisplayInfo()
{
    display_info_ = not display_info_;
}

geometry_msgs::msg::Point Mesh::getReceiverPoint()
{
    return receiver_position_;
}

visualization_msgs::msg::InteractiveMarker Mesh::getInteractiveMarker()
{
    return int_marker_;
}

void Mesh::setEchoColour(rviz_visual_tools::Colors colour)
{
    echo_colour_ = colour;
}

void Mesh::setInfo(std::string& info)
{
    info_ = info;
}

void Mesh::initialise(const std::string& base_frame)
{
    auto file_path = "file://" + ament_index_cpp::get_package_share_directory("visually");
    
    switch (entity_type_)
    {
    case vehicle:
        model_file_ = file_path + "/meshes/car.obj";
        relative_receiver_position_[0] = 0;
        relative_receiver_position_[1] = 0;
        relative_receiver_position_[2] = 0;
        break;
    case rsu:
        model_file_ = file_path + "/meshes/rsu.obj";
        relative_receiver_position_[0] = 0;
        relative_receiver_position_[1] = 0;
        relative_receiver_position_[2] = 14 * scale_;
        break;
    case cloud:
        model_file_ = file_path + "/meshes/server.obj";
        relative_receiver_position_[0] = 0;
        relative_receiver_position_[1] = 2.5 * scale_;
        relative_receiver_position_[2] = 0;
        break;
    }

    line_1_.header.frame_id = line_2_.header.frame_id = line_3_.header.frame_id = base_frame;
    line_1_.ns = line_2_.ns = "spath"; line_3_.ns = "dpath";
    line_1_.id = 0; line_2_.id = 1; line_3_.id = 0;
    line_1_.type = line_2_.type = line_3_.type = line_1_.LINE_STRIP;
    line_1_.action = line_2_.action = line_3_.action = line_1_.ADD;
    line_1_.scale.x = line_1_.scale.y = line_1_.scale.z = 0.125;
    line_2_.scale.x = line_2_.scale.y = line_2_.scale.z = 0.125;
    line_3_.scale.x = line_3_.scale.y = line_3_.scale.z = 0.125;
}

void Mesh::buildInteractiveMarker(const std::string& base_frame)
{
    int_marker_.header.frame_id = base_frame;
    int_marker_.pose = pose_;
    int_marker_.scale = 2.0 * scale_;
    int_marker_.name = id_;

    visualization_msgs::msg::Marker box_marker;
    box_marker.type = visualization_msgs::msg::Marker::SPHERE;
    box_marker.scale.x = 5.0;
    box_marker.scale.y = 5.0;
    box_marker.scale.z = 5.0;
    box_marker.color.a = 0.0;
    
    visualization_msgs::msg::InteractiveMarkerControl click_control;
    click_control.name = id_;
    click_control.always_visible = true;
    click_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
    click_control.markers.push_back(box_marker);
    int_marker_.controls.push_back(click_control);
}

void Mesh::updateStaticEchoLines(const EchoPoints& echo_points)
{
    line_1_.points = echo_points.points[20];
    line_2_.points = echo_points.points[40];
    for (std::size_t i{}; i < echo_line_points; ++i) {

        line_1_.points[i].x += pose_.position.x;
        line_1_.points[i].y += pose_.position.y;
        line_1_.points[i].z += pose_.position.z;

        line_2_.points[i].x += pose_.position.x;
        line_2_.points[i].y += pose_.position.y;
        line_2_.points[i].z += pose_.position.z;
    }
}

void Mesh::updateMovingEchoLine(const EchoPoints& echo_points)
{
    line_3_.points = echo_points.points[moving_line_layer_ / 5];
    for (std::size_t i{}; i < echo_line_points; ++i) {
        
        line_3_.points[i].x += pose_.position.x;
        line_3_.points[i].y += pose_.position.y;
        line_3_.points[i].z += pose_.position.z;
    }
    moving_line_layer_ = (moving_line_layer_ + moving_line_layer_update_rate_) % (max_echo_line_radius * 10);
}

void Mesh::publishUpdates(const EchoPoints& echo_points)
{
    rvizer_.setLifetime(0.5);
    rvizer_.resetMarkerCounts();
    // Publish display text if applicable
    if (display_info_) {
        rvizer_.publishText(
            text_pose_
            , info_
            , rviz_visual_tools::Colors::WHITE
            , rviz_visual_tools::XXXXLARGE);
    }
    rvizer_.setLifetime(0.0);

    // Publish echo markers for entities other than traffic or pedestrian lights
    if (entity_type_ != EntityType::cloud) {
        updateMovingEchoLine(echo_points);
        line_3_.color = rvizer_.getColor(echo_colour_);
        rvizer_.publishMarker(line_3_);
    }

    rvizer_.trigger();
}

void Mesh::publishBaseUpdates(
    const EchoPoints& echo_points)
{
    rvizer_.publishMesh(pose_, model_file_, colour_, scale_, "mesh", 1);
    shape_msgs::msg::Mesh mesh;

    // Publish echo markers for entities other than traffic or pedestrian lights
    if (entity_type_ != EntityType::cloud) {
        updateStaticEchoLines(echo_points);
        line_1_.color = line_2_.color = rvizer_.getColor(echo_colour_);
        rvizer_.publishMarker(line_1_);
        rvizer_.publishMarker(line_2_);
    }

    if (entity_type_ == EntityType::cloud)
        rvizer_.trigger();
}


Link::Link(
    const std::shared_ptr<rclcpp::Node>& node
    , const std::string& base_frame
    , const std::string& id
    , const double max_dist
    , int counter_update_rate)
    : Entity(node, id, base_frame)
    , colour_(rviz_visual_tools::YELLOW)
    , line_thickness_(0.25)
    , opacity_(0.8)
    , packet_size_(0.6)
    , packet_dist_(27.0)
    , valid_point_i_(false)
    , valid_point_o_(false)
    , counter_(0)
    , counter_update_rate_(counter_update_rate)
    , max_dist_(max_dist)
{
    line_.header.frame_id = spheres_.header.frame_id = base_frame;
    line_.ns = "line"; spheres_.ns = "spheres";
    line_.type = line_.LINE_LIST; spheres_.type = spheres_.SPHERE_LIST;
    line_.action = spheres_.action = line_.ADD;
    line_.scale.x = line_thickness_; spheres_.scale.x = packet_size_;
    line_.scale.y = line_thickness_; spheres_.scale.y = packet_size_;
    line_.scale.z = line_thickness_; spheres_.scale.z = packet_size_;
}

Link::~Link()
{

}

void Link::updateEndpoint(
    const std::string& end_point_id
    , const geometry_msgs::msg::Point& point)
{
    auto found{ id_.find(end_point_id) };
    if (found == 0) {
        point_i_ = point;
        valid_point_i_ = true;
    }
    else if (found != id_.npos) {
        point_o_ = point;
        valid_point_o_ = true;
    }
}

void Link::updateLinkSpecs(
    rviz_visual_tools::Colors colour
    , double thickness
    , double opacity
    , double packet_dist)
{
    colour_ = colour;
    line_thickness_ = thickness;
    opacity_ = opacity;
    packet_dist_ = packet_dist;
}

void Link::publishUpdates()
{
    if (valid_point_i_ and valid_point_o_
            and transforms::dist(point_i_, point_o_) <= max_dist_) {
        line_.points.clear();
        line_.points.push_back(point_i_);
        line_.points.push_back(point_o_);
        line_.color = rvizer_.getColor(colour_);
        rvizer_.publishMarker(line_);

        spheres_.points = getPacketPoints();
        spheres_.color = rvizer_.getColor(colour_);
        rvizer_.publishMarker(spheres_);
    }    
    else if (valid_point_i_ and valid_point_o_) {
        rvizer_.setAlpha(opacity_);

        spheres_.points.clear();
        rvizer_.publishMarker(spheres_);
                
        line_.points.clear();
        line_.points.push_back(point_i_);
        line_.points.push_back(point_o_);
        line_.color = rvizer_.getColor(rviz_visual_tools::WHITE);
        rvizer_.publishMarker(line_);
    }
    rvizer_.trigger();
}

std::vector<geometry_msgs::msg::Point> Link::getPacketPoints()
{
    const int num_packets = std::max(
        1
        , int (transforms::dist(point_i_, point_o_) / packet_dist_));
    
    const double x_step = (point_o_.x - point_i_.x) / num_packets;
    const double y_step = (point_o_.y - point_i_.y) / num_packets;
    const double z_step = (point_o_.z - point_i_.z) / num_packets;

    const double pro = (double) counter_ / num_steps_;
    const double reg = 1 - pro;
    
    std::vector<geometry_msgs::msg::Point> packet_points(2 * num_packets);
    
    packet_points[0].x = point_i_.x + pro * x_step;
    packet_points[0].y = point_i_.y + pro * y_step;
    packet_points[0].z = point_i_.z + pro * z_step;

    packet_points[1].x = point_i_.x + reg * x_step;
    packet_points[1].y = point_i_.y + reg * y_step;
    packet_points[1].z = point_i_.z + reg * z_step;

    for (int i{ 2 }; i < 2 * num_packets; ++i) {
        packet_points[i].x = packet_points[i - 2].x + x_step; 
        packet_points[i].y = packet_points[i - 2].y + y_step; 
        packet_points[i].z = packet_points[i - 2].z + z_step;
    }

    counter_ += counter_update_rate_;
    counter_ %= num_steps_;

    return packet_points;
}

BoundingBox::BoundingBox(
    const double position[3]
    , const double yaw
    , const double dimension[3])
    : creation_time_(std::chrono::steady_clock::now())
{
    initialise(position, yaw, dimension);
}

BoundingBox::BoundingBox(
    const geometry_msgs::msg::Pose& pose
    , const geometry_msgs::msg::Vector3& dimension)
    : creation_time_(std::chrono::steady_clock::now())
{
    double position[3]{};
    double orientation[3]{};
    double dimension_array[3]{
        dimension.x
        , dimension.y
        , dimension.z
    };
    transforms::pose2Array(pose, position, orientation);

    initialise(position, orientation[2], dimension_array);
}

BoundingBox::~BoundingBox()
{

}

void BoundingBox::initialise(
    const double position[3]
    , const double yaw
    , const double dimension[3])
{
    dimension_[0] = dimension[0];
    dimension_[1] = dimension[1];
    dimension_[2] = dimension[2];

    pose_ = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    pose_.translation() = Eigen::Vector3d(
        position[0]
        , position[1]
        , position[2] + dimension[2] / 2);
}


Detection::Detection(
    const std::shared_ptr<rclcpp::Node>& node
    , const std::string& base_frame
    , const std::string& id
    , const std::string& colour
    , int fade_time)
    : Detection(
        node
        , base_frame
        , id
        , transforms::getRvizColour(colour)
        , fade_time)
{

}

Detection::Detection(
    const std::shared_ptr<rclcpp::Node>& node
    , const std::string& base_frame
    , const std::string& id
    , rviz_visual_tools::Colors colour
    , int fade_time)
    : Entity(node, id, base_frame)
    , colour_(colour)
    , fade_time_(fade_time)
    , fill_opacity_(0.2)
{
    
}

Detection::~Detection()
{

}

void Detection::deleteDetections()
{
    bboxes_.clear();
}

void Detection::addDetection(
    const geometry_msgs::msg::Pose& pose
    , const geometry_msgs::msg::Vector3& dimension)
{
    bboxes_.emplace_back(pose, dimension);
}

void Detection::addDetections(
    const std::vector<geometry_msgs::msg::Pose>& poses
    , const std::vector<geometry_msgs::msg::Vector3>& dimensions)
{
    assert(poses.size() == dimensions.size()
        && "Detection::addDetections: The size of poses and dimensions vectors are not equal for bounding boxes.");
    
    std::size_t size{ poses.size() };
    
    for (std::size_t i{}; i < size; ++i)
        bboxes_.emplace_back(poses[i], dimensions[i]);
}

void Detection::addDetections(
    const std::vector<double*>& positions
    , const std::vector<double>& yaws
    , const std::vector<double*>& dimensions)
{
    assert(positions.size() == dimensions.size() && positions.size() == yaws.size()
        && "Detection::addDetections: The size of poses, dimensions and yaws vectors are not equal for bounding boxes.");
    
    std::size_t size{ positions.size() };
    
    for (std::size_t i{}; i < size; ++i)
        bboxes_.emplace_back(positions[i], yaws[i], dimensions[i]);
}

void Detection::publishUpdates()
{
    if (not bboxes_.size())
        return;
    
    rvizer_.resetMarkerCounts();
    rvizer_.deleteAllMarkers();
    for (const auto& bbox : bboxes_) {
        rvizer_.setAlpha(1.0);
        rvizer_.publishWireframeCuboid(
            bbox.pose_
            , bbox.dimension_[0]
            , bbox.dimension_[1]
            , bbox.dimension_[2]
            , colour_);
        rvizer_.setAlpha(fill_opacity_);
        rvizer_.publishCuboid(
            bbox.pose_
            , bbox.dimension_[0]
            , bbox.dimension_[1]
            , bbox.dimension_[2]
            , colour_);
    }
    rvizer_.trigger();

    deleteDetections();
}

} // namespace entites