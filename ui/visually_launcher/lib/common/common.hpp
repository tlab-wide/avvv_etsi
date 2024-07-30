#ifndef COMMON_HPP
#define COMMON_HPP

#include <string>
#include <vector>
#include <regex>
#include <fstream>
#include <cstdlib> // For getting environment variables
#include <iostream>

#include <QListView>
#include <QStringListModel>
#include <QStringList>

#include "rosbag2_cpp/reader.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"


namespace common
{


/**
 * @brief Prepares the analyser package for input and output addresses
 * @param input_rosbags
 * @param input_pcaps
 * @param output_folder
 */
void setAnalyseConf(
        const std::string& input_rosbags
        , const std::string& input_pcaps
        , const std::string& output_folder);


/**
 * @brief Attempts to read all topics inside the rosbag file specified by the given path
 * @param file_path Absolute path to the ROSBAG file
 * @returns A list of all available topics inside the rosbag
 * @throws An instance of std::runtime_error if there was a problem opening the file
 */
std::vector<std::string> getAllTopicsOfRosbag(
        const std::string& file_path);


/**
 * @brief Fills the rsu_ids and obu_ids with inferred rsu and obu IDs among the given topics
 * @note Only the RSU_#/tf and OBU_#/tf topics are subject to consideration
 * @param topics
 * @param rsu_ids
 * @param obu_ids
 */
void analyseRosbagTopics(
        const std::vector<std::string>& topics
        , QStringList& rsu_ids
        , QStringList& obu_ids);


/**
 * @brief Edits a file with the given absolute path, replaces the given string in place of the
 * determined line.
 * @param path
 * @param to_replace The text that will be overwritten
 * @param replacing The line that sits in place of 'to_string'
 * @throws std::runtime_error if there is a problem with opening the file
 */
void editFile(
        const std::string& path
        , const std::string& to_replace
        , const std::string& replacing);


/**
 * @brief Updates the list of topics in the visually launch file
 * @param topics
 * @throws std::runtime_error if there is a problem with opening the file
 */
void updateTopicsLaunch(
        const std::vector<std::string>& topics);


/**
 * @brief Gets the suitable topic and name for a display type in RViz conf file based on the given topic
 * @param topic
 * @param rviz_topic
 * @param rviz_name
 */
void getRvizTopicName(
        const std::string& topic
        , std::string& rviz_topic
        , std::string& rviz_name);


/**
 * @brief Updates the display topics in the RViz configuration file
 * @param topics
 */
void updateGeneralTopicsRviz(
        const std::vector<std::string>& topics);


/**
 * @brief Checks whether the selected value for the network parameters tab is valid
 * @note A selection is valid if there are no duplicate values among the selections (except for the 'None' values)
 * @param selecttion1
 * @param selecttion2
 * @param selecttion3
 * @param selecttion4
 * @returns true if the selection is valid, false otherwise
 */
bool isSelectionValid(
        int selection1
        , int selection2
        , int selection3
        , int selection4);


/**
 * @brief Maintains the range of different network parameters the user selects
 */
struct NetworkParameterRanges
{
    std::string delay_best{};
    std::string delay_worst{};
    std::string jitter_best{};
    std::string jitter_worst{};
    std::string rssi_best{};
    std::string rssi_worst{};
    std::string packet_loss_best{};
    std::string packet_loss_worst{};
};


/**
 * @brief Validates the selected network ranges
 * @param ranges
 * @returns true if the selection is valid, false otherwise
 */
bool validateNetworkRanges(
        const NetworkParameterRanges& ranges);


/**
 * @param index The index of the network parameter
 * @returns The string corresponding to the given index
 */
std::string getNetworkAttributeRepresentative(int index);


/**
 * @param index The index of the network parameter
 * @returns The string corresponding to the given index
 */
std::string getNetworkAttribute(int index);


/**
 * @brief Writes the selected general network parameters and their ranges into the launch file
 * @param delay
 * @param jitter
 * @param rssi
 * @param packet_loss
 * @param ranges
 * @throws std::runtime_error if there is a problem with opening the file
 */
void updateNetworkGeneralLaunch(
        int delay
        , int jitter
        , int rssi
        , int packet_loss
        , const NetworkParameterRanges& ranges);


/**
 * @brief Updates the display topics related to offline heatmaps in the RViz configuration file
 * @param rsu_id The ID of the RSU involved in the heatmap
 * @param obu_id The ID of the OBU involved in the heatmap
 * @param include If true, will write the heatmap configurations into RViz conf, otherwise will remove the configurations from RViz conf
 */
void updateOfflineHeatmapTopicsRviz(
        const std::string& rsu_id
        , const std::string& obu_id
        , bool include);


/**
 * @brief Updates the display topics related to online heatmaps in the RViz configuration file
 * @param topics
 * @param include
 */
void updateOnlineHeatmapTopicsRviz(
        const std::vector<std::string>& topics
        , bool include);


/**
 * @brief Writes the selected heatmap and real time graph parameters for the target entities into the launch file
 * @param rsu_obu_con_dist_range The maximum connection distance between any RSU and any OBU
 * @param target_rsu_id
 * @param target_obu_id
 * @param offline_heatmap_path
 * @param offline_heatmap_attribute
 * @param display_real_time_heatmap
 * @param display_real_time_graphs
 */
void updateNetworkTargetedLaunch(
        const std::string& rsu_obu_con_dist_range
        , const std::string& target_rsu_id
        , const std::string& target_obu_id
        , const std::string& offline_heatmap_path
        , int offline_heatmap_attribute
        , bool display_real_time_heatmap
        , bool display_real_time_graphs);


/**
 * @brief Writes the absolute pointcloud file path into the launch file
 * @param pointcloud_path
 * @throws std::runtime_error if there is a problem with opening the file
 */
void updatePointcloudPathLaunch(
        const QString& pointcloud_path);


/**
 * @brief Updates the display topics related to pointcloud in the RViz configuration file
 * @param include
 */
void updatePointcloudTopicsRviz(bool include);


/**
 * @brief Writes the absolute lanelet file path into the launch file
 * @param lanelet_path
 * @throws std::runtime_error if there is a problem with opening the file
 */
void updateLaneletPathLaunch(
        const QString& lanelet_path);


/**
 * @brief Updates the display topics related to lanelet in the RViz configuration file
 * @param include
 */
void updateLaneletTopicsRviz(bool include);


/**
 * @brief Updates the lanelet and pointcloud offsets in the launch file
 * @param x
 * @param y
 * @param z
 */
void updateMapOffsetLaunch(
        const QString& x
        , const QString& y
        , const QString& z);


/**
 * @brief Updates initial focal point of RViz
 * @param x
 * @param y
 * @param z
 */
void updateFocalRviz(
        const QString& x
        , const QString& y
        , const QString& z);


} // namespace common

#endif // COMMON_HPP
