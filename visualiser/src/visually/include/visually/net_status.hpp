/**
 * @author AmirInt <amir.intelli@gmail.com>
 * @brief Provides tools to map network characteristics to visualisation features
*/

#ifndef VISUALLY__NET_STATUS_HPP_
#define VISUALLY__NET_STATUS_HPP_

// C++
#include <string>
#include <functional>
#include <cmath>

// Package
#include <cpm_ros_msgs/msg/network_status.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>


namespace net_status
{

/**
 * @brief Keeps track of the upper and lower bound to each network parameter 
*/
struct NetParamRanges
{
    double delay_best{};
    double delay_worst{};
    double jitter_best{};
    double jitter_worst{};
    std::int64_t rssi_best{};
    std::int64_t rssi_worst{};
    double packet_loss_best{};
    double packet_loss_worst{};
};

/**
 * @brief Used to determine which link feature and characteristics represents which Network field
*/
enum NetworkField
{

delay = 0,
jitter,
rssi,
packetLoss,
none,
maxNetworkFields,

};

/**
 * @brief Converts integer values to NetworkField enumerators
 * @param intput The integer input
 * @returns The corresponding NetworkField enumerator to the given input
*/
net_status::NetworkField intToNetworkField(int intput);


/**
 * @brief Provides tools to convert network parameter values to 
 * link features like colour, opacity, etc
*/
class NetStatusRepr
{
    public:
        /**
         * @brief Constructor
         * @param net_param_ranges The upper/lower bounds to network parameters
        */
        NetStatusRepr(const NetParamRanges& net_param_ranges);

        /**
         * @brief Deconstructor
        */
        ~NetStatusRepr();

        // The functions that handle link colour specification
        const std::function<rviz_visual_tools::Colors(const cpm_ros_msgs::msg::NetworkStatus&)> getColour[maxNetworkFields]{
            [this](const cpm_ros_msgs::msg::NetworkStatus& ns) -> rviz_visual_tools::Colors {
                int index{ static_cast<int>((ns.delay - net_param_ranges_.delay_best)
                        * colours_num_
                        / (net_param_ranges_.delay_worst - net_param_ranges_.delay_best)) };
                
                return colours_[std::min(std::max(index, 0), colours_num_ - 1)];
            }
            , [this](const cpm_ros_msgs::msg::NetworkStatus& ns) -> rviz_visual_tools::Colors {
                int index{ static_cast<int>((ns.jitter - net_param_ranges_.jitter_best)
                        * colours_num_
                        / (net_param_ranges_.jitter_worst - net_param_ranges_.jitter_best)) };
                
                return colours_[std::min(std::max(index, 0), colours_num_ - 1)];
            }
            , [this](const cpm_ros_msgs::msg::NetworkStatus& ns) -> rviz_visual_tools::Colors {
                int index{ static_cast<int>((ns.rssi - net_param_ranges_.rssi_worst)
                        * colours_num_
                        / (net_param_ranges_.rssi_best - net_param_ranges_.rssi_worst)) };
                
                return colours_[std::min(std::max(index, 0), colours_num_ - 1)];
            }
            , [this](const cpm_ros_msgs::msg::NetworkStatus& ns) -> rviz_visual_tools::Colors {
                int index{ static_cast<int>((ns.packet_loss - net_param_ranges_.packet_loss_best)
                        * colours_num_
                        / (net_param_ranges_.packet_loss_worst - net_param_ranges_.packet_loss_best)) };
                
                return colours_[std::min(std::max(index, 0), colours_num_ - 1)];
            }
            , [this](const cpm_ros_msgs::msg::NetworkStatus&) -> rviz_visual_tools::Colors {
                return colours_[2]; // Yellow
            }
        };

        // The functions that handle link thickness specification
        const std::function<double(const cpm_ros_msgs::msg::NetworkStatus&)> getThickness[maxNetworkFields]{
            [this](const cpm_ros_msgs::msg::NetworkStatus& ns) -> double {
                double thickness{ thickness_coef_ * (net_param_ranges_.delay_worst - ns.delay)
                    / (net_param_ranges_.delay_worst - net_param_ranges_.delay_best) };
                return std::max(std::min(thickness, 0.5), 0.1);
            }
            , [this](const cpm_ros_msgs::msg::NetworkStatus& ns) -> double {
                double thickness{ thickness_coef_ * (net_param_ranges_.jitter_worst - ns.jitter)
                    / (net_param_ranges_.jitter_worst - net_param_ranges_.jitter_best) };
                return std::max(std::min(thickness, 0.5), 0.1);
            }
            , [this](const cpm_ros_msgs::msg::NetworkStatus& ns) -> double {
                double thickness{ static_cast<double>(thickness_coef_ * (ns.rssi - net_param_ranges_.rssi_worst)
                    / (net_param_ranges_.rssi_best - net_param_ranges_.rssi_worst)) };
                return std::max(std::min(thickness, 0.5), 0.1);
            }
            , [this](const cpm_ros_msgs::msg::NetworkStatus& ns) -> double {
                double thickness{ thickness_coef_ * (net_param_ranges_.packet_loss_worst - ns.packet_loss)
                    / (net_param_ranges_.packet_loss_worst - net_param_ranges_.packet_loss_best) };
                return std::max(std::min(thickness, 0.5), 0.1);
            }
            , [this](const cpm_ros_msgs::msg::NetworkStatus&) -> double {
                return 0.25;
            }
        };

        // The functions that handle link packet density specification
        const std::function<double(const cpm_ros_msgs::msg::NetworkStatus&)> getPacketDensity[maxNetworkFields]{
            [this](const cpm_ros_msgs::msg::NetworkStatus& ns) -> double {
                double relval{ (ns.delay - net_param_ranges_.delay_best)
                    / (net_param_ranges_.delay_worst - net_param_ranges_.delay_best) };

                return packet_density_min_ + packet_density_coef_ * std::max(std::min(relval, 1.0), 0.0);
            }
            , [this](const cpm_ros_msgs::msg::NetworkStatus& ns) -> double {
                double relval{ (ns.jitter - net_param_ranges_.jitter_best)
                    / (net_param_ranges_.jitter_worst - net_param_ranges_.jitter_best) };

                return packet_density_min_ + packet_density_coef_ * std::max(std::min(relval, 1.0), 0.0);
            }
            , [this](const cpm_ros_msgs::msg::NetworkStatus& ns) -> double {
                double relval{ static_cast<double>(ns.rssi - net_param_ranges_.rssi_worst)
                    / (net_param_ranges_.rssi_best - net_param_ranges_.rssi_worst) };

                return packet_density_min_ + packet_density_coef_ * std::max(std::min(relval, 1.0), 0.0);
            }
            , [this](const cpm_ros_msgs::msg::NetworkStatus& ns) -> double {
                double relval{ (ns.packet_loss - net_param_ranges_.packet_loss_best)
                    / (net_param_ranges_.packet_loss_worst - net_param_ranges_.packet_loss_best) };

                return packet_density_min_ + packet_density_coef_ * std::max(std::min(relval, 1.0), 0.0);
            }
            , [this](const cpm_ros_msgs::msg::NetworkStatus&) -> double {
                return packet_density_min_ + packet_density_coef_ * 0.5;
            }
        };

        // The functions that handle link opacity specification
        const std::function<double(const cpm_ros_msgs::msg::NetworkStatus&)> getOpacity[maxNetworkFields]{
            [this](const cpm_ros_msgs::msg::NetworkStatus& ns) -> double {
                double opacity{ (net_param_ranges_.delay_worst - ns.delay)
                    / (net_param_ranges_.delay_worst - net_param_ranges_.delay_best) };
                return std::max(std::min(opacity, 1.0), 0.1);
            }
            , [this](const cpm_ros_msgs::msg::NetworkStatus& ns) -> double {
                double opacity{ (net_param_ranges_.jitter_worst - ns.jitter)
                    / (net_param_ranges_.jitter_worst - net_param_ranges_.jitter_best) };
                return std::max(std::min(opacity, 1.0), 0.1);
            }
            , [this](const cpm_ros_msgs::msg::NetworkStatus& ns) -> double {
                double opacity{ static_cast<double>((ns.rssi - net_param_ranges_.rssi_worst)
                    / (net_param_ranges_.rssi_best - net_param_ranges_.rssi_worst)) };
                return std::max(std::min(opacity, 1.0), 0.1);
            }
            , [this](const cpm_ros_msgs::msg::NetworkStatus& ns) -> double {
                double opacity{ (net_param_ranges_.packet_loss_worst - ns.packet_loss)
                    / (net_param_ranges_.packet_loss_worst - net_param_ranges_.packet_loss_best) };
                return std::max(std::min(opacity, 1.0), 0.1);
            }
            , [this](const cpm_ros_msgs::msg::NetworkStatus&) -> double {
                return 1.0;
            }
        };
    
    private:
        // The upper/lower bounds to network parameters
        NetParamRanges net_param_ranges_;

        // The number of colours
        const int colours_num_{ 5 };

        // The coefficient used to retrieve the proper thickness
        const double thickness_coef_{ 0.3 };

        // The coefficient used to retrieve the proper packet density
        const double packet_density_coef_{ 50.0 };
        // The minimum packet density value
        const double packet_density_min_{ 2.0 };

        // The colours used for links
        const rviz_visual_tools::Colors colours_[5]{
            rviz_visual_tools::GREEN
            , rviz_visual_tools::LIME_GREEN
            , rviz_visual_tools::YELLOW
            , rviz_visual_tools::ORANGE
            , rviz_visual_tools::RED
        };
};

} // namespace net_status

#endif // VISUALLY__HEATMAP_HPP_