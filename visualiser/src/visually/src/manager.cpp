#include <visually/manager.hpp>


namespace manager
{


Visualiser::Visualiser(
    std::shared_ptr<rclcpp::Node>& node
    , const std::string& base_frame
    , net_status::NetworkField link_colour
    , net_status::NetworkField link_thickness
    , net_status::NetworkField link_packet_density
    , net_status::NetworkField link_opacity
    , const net_status::NetParamRanges& ranges
    , double rsu_obu_con_dist)
    : link_colour_(link_colour)
    , link_thickness_(link_thickness)
    , link_packet_density_(link_packet_density)
    , link_opacity_(link_opacity)
    , node_(node)
    , art_(node, base_frame)
    , net_status_repr_(ranges)
    , rsu_obu_con_dist_(rsu_obu_con_dist)
{

}

Visualiser::~Visualiser()
{

}

void Visualiser::analyseTopic(
    const std::string& topic
    , bool add_online_heatmap)
{
    using CPM = cpm_ros_msgs::msg::CPMMessage;
    using CPMN = cpm_ros_msgs::msg::CPMN;
    using DetObj = autoware_auto_perception_msgs::msg::PredictedObjects;
    using TF = tf2_msgs::msg::TFMessage;
    using NetSta = cpm_ros_msgs::msg::NetworkStatus;

    static const std::regex rsu_tf_rgx{ "/(RSU_\\d+)/tf" };
    static const std::regex rsu_cpm_rgx{ "/(RSU_\\d+)/cpm" };
    static const std::regex rsu_detobj_rgx{ "/(RSU_\\d+)/detected_object" };
    static const std::regex obu_tf_rgx{ "/(OBU_\\d+)/tf" };
    static const std::regex obu_detobj_rgx{ "/(OBU_\\d+)/detected_object" };
    static const std::regex obu_cpmn_rgx{ "/(OBU_\\d+)/(RSU_\\d+)/cpmn" };
    static const std::regex obu_netsta_rgx{ "/(OBU_\\d+)/(RSU_\\d+)/network_status" };

    std::smatch match_results;
    if (std::regex_search(topic, match_results, obu_cpmn_rgx)) {
        auto new_cpmn_id{ match_results[1].str() + "_" + match_results[2].str() + "_cpmn" };
        art_.addDetection(new_cpmn_id, obu_received_colour_);
        cpmn_subscriptions_.push_back(
            node_->create_subscription<CPMN>(
                topic
                , 10
                /**
                 * @todo Figure out why std::bind() doesn't work
                */
                , [this, new_cpmn_id](const CPMN& msg) -> void {
                    cpmDetectionMessageCallback(msg.cpm, new_cpmn_id);
                })
        );
        if (add_online_heatmap) {
            art_.addOnlineHeatmap(
                new_cpmn_id + "_delay"
                , net_status_repr_
                , net_status::delay);
            art_.addOnlineHeatmap(
                new_cpmn_id + "_jitter"
                , net_status_repr_
                , net_status::jitter);
            art_.addOnlineHeatmap(
                new_cpmn_id + "_rssi"
                , net_status_repr_
                , net_status::rssi);
            art_.addOnlineHeatmap(
                new_cpmn_id + "_packet_loss"
                , net_status_repr_
                , net_status::packetLoss);
            online_heatmap_subscriptions_.push_back(
            node_->create_subscription<cpm_ros_msgs::msg::CPMN>(
                topic
                , 10
                , [this, new_cpmn_id](const cpm_ros_msgs::msg::CPMN& msg) -> void {
                    onlineHeatmapCallback(msg, new_cpmn_id);
                })
            );
        }
    }
    else if (std::regex_search(topic, match_results, obu_netsta_rgx)) {
        auto new_net_status_id { match_results[1].str() + '_'  + match_results[2].str() + "_link" };
        art_.addLink(new_net_status_id, rsu_obu_con_dist_);
        netsta_subscriptions_.push_back(
            node_->create_subscription<NetSta>(
                topic
                , 10
                /**
                 * @todo Figure out why the following doesn't work
                */
                // , std::bind(
                //     &Visualiser::networkStatusMessageCallback
                //     , this
                //     , std::placeholders::_1
                //     , match_results[1].str()
                //     , match_results[2].str()))
                , [this, new_net_status_id, match_results](const NetSta& msg) -> void {
                    networkStatusMessageCallback(
                        msg
                        , new_net_status_id
                        , match_results[1].str()
                        , match_results[2].str());
                })
        );        
    }
    else if (std::regex_search(topic, match_results, rsu_tf_rgx)) {
        auto new_rsu_id{ match_results[1].str() };
        art_.addRsu(new_rsu_id, rsu_colour_);
        tf_subscriptions_.push_back(
            node_->create_subscription<TF>(
                topic
                , 10
                /**
                 * @todo Figure out why the following doesn't work
                */
                // , std::bind(
                //     &Visualiser::rsuTfMessageCallback
                //     , this
                //     , std::placeholders::_1
                //     , match_results[1].str()))
                , [this, new_rsu_id](const TF& msg) -> void {
                    rsuTfMessageCallback(msg, new_rsu_id);
                })
        );
    }
    else if (std::regex_search(topic, match_results, rsu_cpm_rgx)) {
        auto new_rsu_det_id{ match_results[1].str() + "_cpm" };
        art_.addDetection(new_rsu_det_id, rsu_sent_vehicle_colour_);
        cpm_subscriptions_.push_back(
            node_->create_subscription<CPM>(
                topic
                , 10
                /**
                 * @todo Figure out why std::bind() doesn't work
                */
                , [this, new_rsu_det_id](const CPM& msg) -> void {
                    cpmDetectionMessageCallback(msg, new_rsu_det_id);
                })
        );
    }
    else if (std::regex_search(topic, match_results, rsu_detobj_rgx)) {
        auto new_rsu_det_id{ match_results[1].str() + "_detections" };
        art_.addDetection(new_rsu_det_id, rsu_detected_colour_, true);
        detobj_subscriptions_.push_back(
            node_->create_subscription<DetObj>(
                topic
                , 10
                /**
                 * @todo Figure out why the following doesn't work
                */
                // , std::bind(
                //     &Visualiser::rsuDetectedObjectsMessageCallback
                //     , this
                //     , std::placeholders::_1
                //     , match_results[1].str()))
                , [this, new_rsu_det_id](const DetObj& msg) -> void {
                    detectionMessageCallback(msg, new_rsu_det_id);
                })
        );
    }
    else if (std::regex_search(topic, match_results, obu_tf_rgx)) {
        auto new_obu_id{ match_results[1].str() };
        art_.addVehicle(new_obu_id, obu_colour_);
        tf_subscriptions_.push_back(
            node_->create_subscription<TF>(
                topic
                , 10
                /**
                 * @todo Figure out why the following doesn't work
                */
                // , std::bind(
                //     &Visualiser::obuTfMessageCallback
                //     , this
                //     , std::placeholders::_1
                //     , match_results[1].str()))
                , [this, new_obu_id](const TF& msg) -> void {
                    obuTfMessageCallback(msg, new_obu_id);
                })
        );
    }
    else if (std::regex_search(topic, match_results, obu_detobj_rgx)) {
        auto new_obu_det_id{ match_results[1].str() + "_detections" };
        art_.addDetection(new_obu_det_id, obu_detected_colour_, true);
        detobj_subscriptions_.push_back(
            node_->create_subscription<DetObj>(
                topic
                , 10
                /**
                 * @todo Figure out why the following doesn't work
                */
                // , std::bind(
                //     &Visualiser::obuDetectedObjectsMessageCallback
                //     , this
                //     , std::placeholders::_1
                //     , match_results[1].str()))
                , [this, new_obu_det_id](const DetObj& msg) -> void {
                    detectionMessageCallback(msg, new_obu_det_id);
                })
        );
    }
}

void Visualiser::addOfflineHeatmap(
    const std::string& offline_heatmap_path
    , const std::string& rsu_id
    , const std::string& obu_id
    , int network_attr)
{
    std::string id{obu_id + '_' + rsu_id + "_offline_heatmap"};
    art_lock_.lock();
    art_.addOfflineHeatmap(
        id
        , offline_heatmap_path
        , net_status_repr_
        , network_attr);
    art_lock_.unlock();
}

uint8_t Visualiser::maxLabel(
    const std::vector<autoware_auto_perception_msgs::msg::ObjectClassification>& classifications)
{
    std::size_t max_index{};
    double max_probability{ classifications.at(0).probability };
    for (std::size_t i{ 1 }; i < classifications.size(); ++i)
        if (classifications.at(i).probability > max_probability) {
            max_index = i;
            max_probability = classifications.at(i).probability;
        }
    
    return classifications.at(max_index).label;
}

void Visualiser::rsuTfMessageCallback(
    const tf2_msgs::msg::TFMessage& msg
    , const std::string& rsu_id)
{
    geometry_msgs::msg::Transform new_transform{ msg.transforms.at(0).transform };
    geometry_msgs::msg::Pose new_pose;
    new_pose.position.x = new_transform.translation.x;
    new_pose.position.y = new_transform.translation.y;
    new_pose.position.z = new_transform.translation.z;
    new_pose.orientation = new_transform.rotation;
    art_lock_.lock();
    art_.updateRsuPose(rsu_id, new_pose);
    art_lock_.unlock();
}

void Visualiser::detectionMessageCallback(
    const autoware_auto_perception_msgs::msg::PredictedObjects& msg
    , const std::string& det_id)
{
    using Class = autoware_auto_perception_msgs::msg::ObjectClassification;
    std::vector<geometry_msgs::msg::Pose> poses;
    std::vector<geometry_msgs::msg::Vector3> dims;

    for (const auto& pred_obj : msg.objects)
        if (maxLabel(pred_obj.classification) == Class::CAR) {
            dims.push_back(pred_obj.shape.dimensions);
            poses.push_back(pred_obj.kinematics.initial_pose_with_covariance.pose);
        }

    art_.updateDetection(det_id);
    art_.updateDetection(det_id, poses, dims);
}

void Visualiser::obuTfMessageCallback(
    const tf2_msgs::msg::TFMessage& msg
    , const std::string& obu_id)
{
    geometry_msgs::msg::Transform new_transform{ msg.transforms.at(0).transform };
    geometry_msgs::msg::Pose new_pose;
    new_pose.position.x = new_transform.translation.x;
    new_pose.position.y = new_transform.translation.y;
    new_pose.position.z = new_transform.translation.z;
    new_pose.orientation = new_transform.rotation;
    art_lock_.lock();
    art_.updateVehiclePose(obu_id, new_pose);
    art_lock_.unlock();
}

void Visualiser::cpmDetectionMessageCallback(
    const cpm_ros_msgs::msg::CPMMessage& msg
    , const std::string& det_id)
{
    double x, y;
    transforms::toMgrs(
        msg.cpm.cpm_parameters.management_container.reference_position.longitude * geo_scale_
        , msg.cpm.cpm_parameters.management_container.reference_position.latitude * geo_scale_
        , x
        , y);

    double rsu_yaw = msg.cpm.cpm_parameters.
        station_data_container.
        originating_vehicle_container.
        heading * angle_scale_;

    double trfmx[4][4];
    double p[3]{ x, y, 0.0 };
    double o[3]{ 0.0, 0.0, rsu_yaw };
    transforms::getTransformMatrix(trfmx, p, o);

    std::vector<double*> positions;
    std::vector<double> yaws;
    std::vector<double*> dims;

    for (const auto& po : msg.cpm.cpm_parameters.perceived_object_container.perceived_objects) {
        const double position[3]{
            po.x_distance * distance_scale_
            , po.y_distance * distance_scale_
            , po.z_distance * distance_scale_
        };

        double result_position[3]{};
        transforms::transformPoint(result_position, position, trfmx);

        positions.push_back(result_position);

        const double orientation[3]{
            po.roll_angle * angle_scale_
            , po.pitch_angle * angle_scale_
            , po.yaw_angle * angle_scale_
        };

        yaws.push_back(orientation[2]);

        double dimension[3]{
            po.planar_object_dimension2 * dimension_scale_
            , po.planar_object_dimension1 * dimension_scale_
            , po.vertical_object_dimension * dimension_scale_
        };

        dims.push_back(dimension);
    }

    art_lock_.lock();
    art_.updateDetection(det_id);
    art_.updateDetection(det_id, positions, yaws, dims);
    art_lock_.unlock();
}

void Visualiser::networkStatusMessageCallback(
    const cpm_ros_msgs::msg::NetworkStatus& msg
    , const std::string& id
    , const std::string& obu_id
    , const std::string& rsu_id)
{
    art_lock_.lock();

    // Update the target link
    art_.updateLinkSpec(
        id
        , net_status_repr_.getColour[link_colour_](msg)
        , net_status_repr_.getThickness[link_thickness_](msg)
        , net_status_repr_.getOpacity[link_opacity_](msg)
        , net_status_repr_.getPacketDensity[link_packet_density_](msg));

    art_.setObuRsuPacketLoss(obu_id, rsu_id, msg.packet_loss);

    cpm_ros_msgs::msg::NetworkStatus msg_dup{};

    msg_dup.packet_loss = art_.getAvgObuRsuPacketLoss(obu_id);

    // Update the vehicle whose link this is
    art_.updateVehicleEchoColour(
        obu_id
        , net_status_repr_.getColour[net_status::packetLoss](msg_dup));
    
    art_lock_.unlock();
}

void Visualiser::onlineHeatmapCallback(
    const cpm_ros_msgs::msg::CPMN& msg
    , const std::string& id)
{
    double x{ msg.obu_tf.transforms.at(0).transform.translation.x };
    double y{ msg.obu_tf.transforms.at(0).transform.translation.y };
    double z{ msg.obu_tf.transforms.at(0).transform.translation.z };

    art_.addToOnlineHeatmap(id + "_delay", x, y, z, msg.network_status.delay);
    art_.addToOnlineHeatmap(id + "_jitter", x, y, z, msg.network_status.jitter);
    art_.addToOnlineHeatmap(id + "_rssi", x, y, z, msg.network_status.rssi);
    art_.addToOnlineHeatmap(id + "_packet_loss", x, y, z, msg.network_status.packet_loss);
}

} // namespace common