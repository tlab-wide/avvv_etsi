#include <visually/rviz_tools.hpp>


namespace rviz_tools
{

RvizTools::RvizTools(
    std::shared_ptr<rclcpp::Node>& node
    , const std::string& base_frame)
    : node_(node)
    , base_frame_(base_frame)
    , display_timer_(node_->create_wall_timer(
        display_period_
        , std::bind(&RvizTools::display, this)))
    , base_display_timer_(node_->create_wall_timer(
        2s
        , std::bind(&RvizTools::baseDisplay, this)))
    , int_server_("/avvv_interactives", node)
{
    
}

RvizTools::~RvizTools()
{
    
}

void RvizTools::addVehicle(
    const std::string& id
    , const std::string& colour
    , double opacity)
{
    if (obus_.find(id) == obus_.end()) {
        entities::Mesh new_vehicle{
            node_
            , base_frame_
            , entities::EntityType::vehicle
            , id
            , colour
            , 1.0
            , opacity
            , display_period_.count() / display_period_base_
        };

        obus_.emplace(id, new_vehicle);

        callbacks_.emplace(id, [this, id](interactive_markers::InteractiveMarkerServer::FeedbackConstSharedPtr) {
            obus_.at(id).toggleDisplayInfo();
        });

        int_server_.insert(new_vehicle.getInteractiveMarker());
        int_server_.setCallback(
            new_vehicle.getInteractiveMarker().name
            , callbacks_.at(id)
            , visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK);
    }
}

void RvizTools::removeVehicle(
    const std::string& id)
{
    try {
        obus_.erase(id);
    }
    catch (std::out_of_range&) {}
}

void RvizTools::updateVehiclePose(
    const std::string& id
    , const geometry_msgs::msg::Pose& pose)
{
    try {
        auto& vehicle{ obus_.at(id) };
        vehicle.setPose(pose);
        updateLinkEndpoints(id, vehicle.getReceiverPoint());
        int_server_.setPose(id, pose);
    }
    catch (std::out_of_range&) {
    }
}

void RvizTools::updateVehicleEchoColour(
    const std::string& id
    , rviz_visual_tools::Colors colour)
{
    try {
        auto& vehicle{ obus_.at(id) };
        vehicle.setEchoColour(colour);
    }
    catch (std::out_of_range&) {
    }
}

void RvizTools::addRsu(
    const std::string& id
    , const std::string& colour
    , double opacity)
{
    if (rsus_.find(id) == rsus_.end()) {
        entities::Mesh new_rsu{
            node_
            , base_frame_
            , entities::EntityType::rsu
            , id
            , colour
            , 1.0
            , opacity
            , display_period_.count() / display_period_base_
        };
        
        rsus_.emplace(id, new_rsu);

        callbacks_.emplace(id, [this, id](interactive_markers::InteractiveMarkerServer::FeedbackConstSharedPtr) {
            rsus_.at(id).toggleDisplayInfo();
        });

        int_server_.insert(new_rsu.getInteractiveMarker());
        int_server_.setCallback(
            new_rsu.getInteractiveMarker().name
            , callbacks_.at(id)
            , visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK);
    }
}

void RvizTools::removeRsu(
    const std::string& id)
{
    try {
        rsus_.erase(id);
    }
    catch (std::out_of_range&) {}
}

void RvizTools::updateRsuPose(
    const std::string& id
    , const geometry_msgs::msg::Pose& pose)
{
    try {
        auto& rsu{ rsus_.at(id) };
        rsu.setPose(pose);
        updateLinkEndpoints(id, rsu.getReceiverPoint());
        int_server_.setPose(id, pose);
    }
    catch (std::out_of_range&) {
    }
}


void RvizTools::addCloud(
    const std::string& id
    , const std::string& colour
    , double opacity)
{
    if (clouds_.find(id) == clouds_.end()) {
        entities::Mesh new_cloud{
            node_
            , base_frame_
            , entities::EntityType::cloud
            , id
            , colour
            , 1.0
            , opacity
        };

        clouds_.emplace(id, new_cloud);

        callbacks_.emplace(id, [this, id](interactive_markers::InteractiveMarkerServer::FeedbackConstSharedPtr) {
            clouds_.at(id).toggleDisplayInfo();
        });

        int_server_.insert(new_cloud.getInteractiveMarker());
        int_server_.setCallback(
            new_cloud.getInteractiveMarker().name
            , callbacks_.at(id)
            , visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK);
    }
}

void RvizTools::removeCloud(
    const std::string& id)
{
    try {
        clouds_.erase(id);
    }
    catch (std::out_of_range&) {}
}

void RvizTools::updateCloudPose(
    const std::string& id
    , const geometry_msgs::msg::Pose& pose)
{
    try {
        auto& cloud{ clouds_.at(id) };
        cloud.setPose(pose);
        updateLinkEndpoints(id, cloud.getReceiverPoint());
        int_server_.setPose(id, pose);
    }
    catch (std::out_of_range&) {
    }
}

void RvizTools::addLink(
    const std::string& id
    , double max_dist)
{
    if (links_.find(id) == links_.end()) {
        entities::Link new_link{
            node_
            , base_frame_
            , id
            , max_dist
            , display_period_.count() / display_period_base_
        };

        links_.emplace(id, new_link);
    }
}

void RvizTools::addDetection(
    const std::string& id
    , const std::string& colour
    , int fade_time)
{
    if (detections_.find(id) == detections_.end()) {
        entities::Detection new_detection{ 
            node_
            , base_frame_
            , id
            , colour
            , fade_time
        };

        detections_.emplace(id, new_detection);
    }
}

void RvizTools::updateDetection(
    const std::string& id)
{
    try {
        detections_.at(id).deleteDetections();
    }
    catch (std::out_of_range&) {
    }
}

void RvizTools::updateDetection(
    const std::string& id
    , const geometry_msgs::msg::Pose& pose
    , const geometry_msgs::msg::Vector3& dimension)
{
    try {
        detections_.at(id).addDetection(pose, dimension);
    }
    catch (std::out_of_range&) {
    }
}

void RvizTools::updateDetection(
    const std::string& id
    , const std::vector<geometry_msgs::msg::Pose>& poses
    , const std::vector<geometry_msgs::msg::Vector3>& dimensions)
{
    try {
        detections_.at(id).addDetections(poses, dimensions);
    }
    catch (std::out_of_range&) {
    }
}

void RvizTools::updateDetection(
    const std::string& id
    , const std::vector<double*>& positions
    , const std::vector<double>& yaws
    , const std::vector<double*>& dimensions)
{
    try {
        detections_.at(id).addDetections(positions, yaws, dimensions);
    }
    catch (std::out_of_range&) {
    }
}

void RvizTools::addOfflineHeatmap(
    const std::string& id
    , const std::string& file_path
    , const net_status::NetStatusRepr& net_status_repr
    , int network_attr)
{
    if (offline_heatmaps_.find(id) == offline_heatmaps_.end()) {
        heatmap::OfflineHeatmap new_offline_heatmap{
            node_
            , id
            , base_frame_
            , net_status_repr
            , network_attr };
        offline_heatmaps_.emplace(id, new_offline_heatmap);
        offline_heatmaps_.at(id).withdrawHeatmapPoints(file_path);
        offline_heatmaps_.at(id).buildMarkers();
    }
}

void RvizTools::addOnlineHeatmap(
    const std::string& id
    , const net_status::NetStatusRepr& net_status_repr
    , int network_attr)
{
    if (online_heatmaps_.find(id) == online_heatmaps_.end()) {
        heatmap::OnlineHeatmap new_online_heatmap{
            node_
            , id
            , base_frame_
            , net_status_repr
            , network_attr };
        online_heatmaps_.emplace(id, new_online_heatmap);
    }
}

void RvizTools::addToOnlineHeatmap(
    const std::string& id
    , double x
    , double y
    , double z
    , double value)
{
    try {
        online_heatmaps_.at(id).addNewOnlineHeatmapPoint(
            x
            , y
            , z
            , value);
    } catch (std::out_of_range&) {}
}

double RvizTools::getAvgObuRsuPacketLoss(const std::string& obu_id)
{
    double avg{};

    for (auto value : obu_rsu_packet_losses_[obu_id]) {
        avg += value.second;
    }

    avg /= obu_rsu_packet_losses_[obu_id].size();

    return avg;
}

void RvizTools::setObuRsuPacketLoss(
    const std::string& obu_id
    , const std::string& rsu_id
    , double value)
{
    obu_rsu_packet_losses_[obu_id][rsu_id] = value;
}

void RvizTools::display()
{
    for (auto& obu : obus_) {
        obu.second.publishBaseUpdates(echo_points_);
        obu.second.publishUpdates(echo_points_);
    }

    for (auto& rsu : rsus_)
        rsu.second.publishUpdates(echo_points_);
    
    for (auto& detection : detections_)
        detection.second.publishUpdates();

    for (auto& link : links_)
        link.second.publishUpdates();

    for (auto& online_heatmap : online_heatmaps_)
        online_heatmap.second.publishUpdates();
    
    int_server_.applyChanges();
}

void RvizTools::baseDisplay()
{
    for (auto& rsu : rsus_)
        rsu.second.publishBaseUpdates(echo_points_);

    for (auto& cloud : clouds_)
        cloud.second.publishBaseUpdates(echo_points_);
    
    for (auto& offline_heatmap : offline_heatmaps_)
        offline_heatmap.second.publishUpdates();
}

void RvizTools::updateLinkEndpoints(
    const std::string& id
    , const geometry_msgs::msg::Point& point)
{
    for (auto& entry : links_)
        entry.second.updateEndpoint(id, point);
}

void RvizTools::updateLinkSpec(
    const std::string& id
    , rviz_visual_tools::Colors colour
    , double line_thickness
    , double opacity
    , double packet_dist)
{
    try {
        links_.at(id).updateLinkSpecs(
            colour
            , line_thickness
            , opacity
            , packet_dist);
    }
    catch (std::out_of_range&) {
    }
}

} // namespace rviz_tools3