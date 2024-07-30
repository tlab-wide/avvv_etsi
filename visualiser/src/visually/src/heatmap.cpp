#include <visually/heatmap.hpp>

namespace heatmap
{

HeatmapRecorder::HeatmapRecorder(
    const std::shared_ptr<rclcpp::Node>& node
    , const std::string& id
    , const std::string& base_frame
    , const net_status::NetStatusRepr& net_status_repr
    , int network_attr
    , double opacity
    , double scale)
    : entities::Entity(node, id, base_frame)
    , net_status_repr_(net_status_repr)
    , network_attr_(net_status::intToNetworkField(network_attr))
    , opacity_(opacity)
    , scale_(scale)
    , marker_id_(0)
{
    rvizer_.setAlpha(opacity_);
}

HeatmapRecorder::~HeatmapRecorder()
{

}

const std::vector<HeatmapPoint>& HeatmapRecorder::getHeatmapPoints() const
{
    return heatmap_points_;
}

void HeatmapRecorder::publishUpdates()
{

}

void HeatmapRecorder::addNewHeatmapPoint(
    double x
    , double y
    , double z
    , rviz_visual_tools::Colors colour)
{
    HeatmapPoint new_hm_point;
    new_hm_point.geo_point.x = x;
    new_hm_point.geo_point.y = y;
    new_hm_point.geo_point.z = z;
    new_hm_point.colour = colour;
    heatmap_points_.push_back(new_hm_point);
}


OfflineHeatmap::OfflineHeatmap(
    const std::shared_ptr<rclcpp::Node>& node
    , const std::string& id
    , const std::string& base_frame
    , const net_status::NetStatusRepr& net_status_repr
    , int network_attr)
    : HeatmapRecorder(
        node
        , id
        , base_frame
        , net_status_repr
        , network_attr
        , 0.4
        , 10.0) // Default opacity and scale
{
    marker_.header.frame_id = base_frame;
    marker_.scale.x = marker_.scale.y = marker_.scale.z = 1.0;
    marker_.type = marker_.TRIANGLE_LIST;
    marker_.action = marker_.ADD;
    marker_.color.a = 1.0;
}

OfflineHeatmap::~OfflineHeatmap()
{

}

void OfflineHeatmap::withdrawHeatmapPoints(
    const std::string& file_path)
{
    using namespace net_status;

    // First withdraw the heatmap point size
    const std::regex grid_size_rgx{ "Grid_Size : (\\d+)" };
    std::smatch match_results;
    
    // The reader of the CSV file provided in the constructor
    csv2::Reader<csv2::delimiter<','>,
                 csv2::quote_character<'"'>,
                 csv2::first_row_is_header<false>,
                 csv2::trim_policy::trim_whitespace> csv_reader;

    if (!csv_reader.mmap(file_path))
        throw std::runtime_error("Error: Could not load the Heatmap CSV file");

    const auto header{ csv_reader.header() };
    std::string value{};
    double value_num{};
    for (const auto row : csv_reader) {
        if (row.length() <= 1)
            continue;
        value.clear();
        row.read_raw_value(value);
        if (value.find('#') != value.npos) {
            if (std::regex_search(value, match_results, grid_size_rgx))
                scale_ = std::stod(match_results[1].str()) / 2 - 0.1;
            continue;
        }
        
        static int index;
        index = 0;
        
        // Data to be written into the new heatmap
        static double heatmap_point_x;
        static double heatmap_point_y;
        static double heatmap_point_z{};
        static rviz_visual_tools::Colors heatmap_colour;
        
        for (const auto cell : row) {
            value.clear();
            cell.read_value(value);
            try {
                value_num = std::stod(value);
            }
            catch (std::invalid_argument&) {
                break;
            }
            switch (index) {
            case 0:
                heatmap_point_x = value_num;
                break;
            case 1:
                heatmap_point_y = value_num;
                break;
            case 2:
                cpm_ros_msgs::msg::NetworkStatus net_status;
                switch (network_attr_) {
                    case delay:
                        net_status.delay = value_num;
                        break;
                    case jitter:
                        net_status.jitter = value_num;
                        break;
                    case rssi:
                        net_status.rssi = value_num;
                        break;
                    default:
                        net_status.packet_loss = value_num;
                }
                heatmap_colour = net_status_repr_.getColour[network_attr_](net_status);
            }
            ++index;
        }

        addNewHeatmapPoint(
            heatmap_point_x
            , heatmap_point_y
            , heatmap_point_z
            , heatmap_colour);
    }
}

void OfflineHeatmap::buildMarkers()
{
    std_msgs::msg::ColorRGBA colour;
    geometry_msgs::msg::Point point;
    for (const auto& heatmap_point : heatmap_points_) {
        point = heatmap_point.geo_point;
        point.x -= scale_;
        point.y -= scale_;
        marker_.points.push_back(point);
        point.x += 2 * scale_;
        marker_.points.push_back(point);
        point.y += 2 * scale_;
        marker_.points.push_back(point);
        marker_.points.push_back(point);
        point.x -= 2 * scale_;
        marker_.points.push_back(point);
        point.y -= 2 * scale_;
        marker_.points.push_back(point);
        colour = rvizer_.getColor(heatmap_point.colour);
        marker_.colors.push_back(colour);
        marker_.colors.push_back(colour);
        marker_.colors.push_back(colour);
        marker_.colors.push_back(colour);
        marker_.colors.push_back(colour);
        marker_.colors.push_back(colour);
    }
}

void OfflineHeatmap::publishUpdates()
{
    rvizer_.publishMarker(marker_);
    rvizer_.trigger();
}


OnlineHeatmap::OnlineHeatmap(
    const std::shared_ptr<rclcpp::Node>& node
    , const std::string& id
    , const std::string& base_frame
    , const net_status::NetStatusRepr& net_status_repr
    , int network_attr)
    : HeatmapRecorder(
        node
        , id
        , base_frame
        , net_status_repr
        , network_attr
        , 1.0
        , 1.0) // The default opacity and scale
    , count_(0)
{
    line_strip_.header.frame_id = base_frame;
    line_strip_.action = line_strip_.ADD;
    line_strip_.type = line_strip_.LINE_STRIP;
    line_strip_.scale.x = line_strip_.scale.y = line_strip_.scale.z = scale_;
    line_strip_.frame_locked = true;
}

OnlineHeatmap::~OnlineHeatmap()
{

}

void OnlineHeatmap::addNewOnlineHeatmapPoint(
    double x
    , double y
    , double z
    , double value)
{
    using namespace net_status;
    cpm_ros_msgs::msg::NetworkStatus net_status;
    switch (network_attr_) {
        case delay:
            net_status.delay = value;
            break;
        case jitter:
            net_status.jitter = value;
            break;
        case rssi:
            net_status.rssi = value;
            break;
        default:
            net_status.packet_loss = value;
    }

    geometry_msgs::msg::Point new_point;
    new_point.x = x;
    new_point.y = y;
    new_point.z = z;

    // If the last point stored is too far away from the new point, 
    // assume a restart and clear the previously stored points
    if (heatmap_points_.size()
             and transforms::dist(
                new_point
                , heatmap_points_.back().geo_point) > 5.0) {
        heatmap_points_.clear();
        count_ = 0;
    }

    addNewHeatmapPoint(
        x
        , y
        , z
        , net_status_repr_.getColour[network_attr_](net_status));
}

void OnlineHeatmap::publishUpdates()
{
    using rviz_visual_tools::Colors; 
    using visualization_msgs::msg::Marker;

    line_strip_.points.clear();
    line_strip_.colors.clear();
    static std::size_t stop;
    stop = heatmap_points_.size();

    // Make an attempt to keep the online heatmap continuous and connected
    for (count_ = (count_ == 0 ? count_ : count_ - 1) ; count_ < stop; ++count_) {
        line_strip_.points.push_back(heatmap_points_[count_].geo_point);
        line_strip_.colors.push_back(rvizer_.getColor(heatmap_points_[count_].colour));
    }

    line_strip_.id = marker_id_;

    rvizer_.publishMarker(line_strip_);

    rvizer_.trigger();

    ++marker_id_;
}

} // namespace heatmap