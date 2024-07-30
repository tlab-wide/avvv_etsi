#include "common.hpp"

namespace common
{


void setAnalyseConf(
        const std::string& input_rosbags
        , const std::string& input_pcaps
        , const std::string& output_folder)
{
    auto confini_file_path{ std::string(std::getenv("AVVV_ETSI_HOME")) +
        "analyser/conf.ini" };
    editFile(confini_file_path, "pcap_files_directory = "
             , std::string("pcap_files_directory = ") + ('"' + input_pcaps + '"'));
    editFile(confini_file_path, "rosbag_files_directory = "
             , std::string("rosbag_files_directory = ") + ('"' + input_rosbags + '"'));
    editFile(confini_file_path, "rosbag_output_address = ",
             std::string("rosbag_output_address = ") + ('"' + output_folder + "/outputs" + '"'));
    editFile(confini_file_path, "report_output_address = ",
             std::string("report_output_address = ") + ('"' + output_folder + "/outputs" + '"'));

    editFile(confini_file_path, "save_graphs_csv = ", "save_graphs_csv = \"True\"");
    editFile(confini_file_path, "save_graphs_image = ", "save_graphs_image = \"True\"");
    editFile(confini_file_path, "showing_graphs = ", "showing_graphs = \"False\"");
}


std::vector<std::string> getAllTopicsOfRosbag(
        const std::string& file_path)
{
    rosbag2_cpp::Reader reader;
    reader.open(file_path);

    auto topic_metadata = reader.get_all_topics_and_types();

    std::vector<std::string> topic_names;
    for (const auto& topic : topic_metadata)
        topic_names.push_back(topic.name);

    return topic_names;
}


void analyseRosbagTopics(
        const std::vector<std::string>& topics,
        QStringList& rsu_ids,
        QStringList& obu_ids)
{
    static const std::regex rsu_rgx{ "/(RSU_\\d+)" };
    static const std::regex obu_rgx{ "/(OBU_\\d+)" };

    std::smatch match_results;
    for (const auto& topic : topics) {
        if (std::regex_search(topic, match_results, rsu_rgx)
                and not rsu_ids.contains(QString::fromStdString(match_results[1].str())))
            rsu_ids.append(QString::fromStdString(match_results[1].str()));
        else if (std::regex_search(topic, match_results, obu_rgx)
                and not obu_ids.contains(QString::fromStdString(match_results[1].str())))
            obu_ids.append(QString::fromStdString(match_results[1].str()));
    }
}


void editFile(
        const std::string& path
        , const std::string& to_replace
        , const std::string& replacing)
{
    std::ifstream file_in(path);
    std::vector<std::string> lines;
    if (!file_in)
        throw std::runtime_error("Error: Could not open file for reading: " + path);
    std::string temp_str;
    while(std::getline(file_in, temp_str)) {
        if (temp_str.find(to_replace) != std::string::npos)
            lines.push_back(replacing + '\n');
        else
            lines.push_back(temp_str + '\n');
    }
    file_in.close();

    std::ofstream file_out(path);
    if (!file_out)
        throw std::runtime_error("Error: Could not overwrite file: " + path);
    for (const auto& line : lines)
        file_out << line;
    file_out.close();
}


void updateTopicsLaunch(
        const std::vector<std::string>& topics)
{
    auto launch_file_path{ ament_index_cpp::get_package_share_directory("visually") + "/launch/" + "visualiser.launch.py"};
    std::string topics_string{};
    for (int i{}; i < topics.size(); ++i) {
        topics_string += '"' + topics[i] + '"';
        if (i != topics.size() - 1)
            topics_string += ", ";
    }
    editFile(launch_file_path, "TOPICS = ", "TOPICS = [" + topics_string + "]");
}


void getRvizTopicName(
        const std::string& topic
        , std::string& rviz_topic
        , std::string& rviz_name)
{
    static const std::regex rsu_tf_rgx{ "/(RSU_\\d+)/tf" };
    static const std::regex rsu_cpm_rgx{ "/(RSU_\\d+)/cpm" };
    static const std::regex rsu_detobj_rgx{ "/(RSU_\\d+)/detected_object" };
    static const std::regex obu_tf_rgx{ "/(OBU_\\d+)/tf" };
    static const std::regex obu_detobj_rgx{ "/(OBU_\\d+)/detected_object" };
    static const std::regex obu_cpmn_rgx{ "/(OBU_\\d+)/(RSU_\\d+)/cpmn" };
    static const std::regex obu_netsta_rgx{ "/(OBU_\\d+)/(RSU_\\d+)/network_status" };

    std::smatch match_results;
    if (std::regex_search(topic, match_results, obu_cpmn_rgx)) {
        rviz_topic = match_results[1].str() + "_" + match_results[2].str() + "_cpmn";
        rviz_name = match_results[1].str() + ':' + match_results[2].str() + " CPMN";
    }
    else if (std::regex_search(topic, match_results, obu_netsta_rgx)) {
        rviz_topic = match_results[1].str() + "_" + match_results[2].str() + "_link";
        rviz_name = match_results[1].str() + ':' + match_results[2].str() + " Link";
    }
    else if (std::regex_search(topic, match_results, rsu_tf_rgx)) {
        rviz_topic = match_results[1].str();
        rviz_name = match_results[1].str();
    }
    else if (std::regex_search(topic, match_results, rsu_cpm_rgx)) {
        rviz_topic = match_results[1].str() + "_cpm";
        rviz_name = match_results[1].str() + " CPM";
    }
    else if (std::regex_search(topic, match_results, rsu_detobj_rgx)) {
        rviz_topic = match_results[1].str() + "_detections";
        rviz_name = match_results[1].str() + " Detections";
    }
    else if (std::regex_search(topic, match_results, obu_tf_rgx)) {
        rviz_topic = match_results[1].str();
        rviz_name = match_results[1].str();
    }
    else if (std::regex_search(topic, match_results, obu_detobj_rgx)) {
        rviz_topic = match_results[1].str() + "_detections";
        rviz_name = match_results[1].str() + " Detections";
    }
}


void updateGeneralTopicsRviz(
        const std::vector<std::string>& topics)
{
    auto rviz_file_path{ ament_index_cpp::get_package_share_directory("visually") + "/rviz/" + "rviz_conf.rviz"};
    std::ifstream file_in(rviz_file_path);

    if (!file_in)
        throw std::runtime_error("Error: Could not open file for reading: " + rviz_file_path);

    std::vector<std::string> lines;

    std::string rviz_topic;
    std::string rviz_name;

    std::string temp_str;
    while(std::getline(file_in, temp_str))
        if (temp_str.find("# General") == std::string::npos)
            lines.push_back(temp_str + '\n');
        else
            break;
    lines.push_back(temp_str + '\n');

    for (const auto& topic: topics) {
        getRvizTopicName(topic, rviz_topic, rviz_name);

        lines.push_back("    - Class: rviz_default_plugins/MarkerArray\n");
        lines.push_back("      Enabled: true\n");
        lines.push_back("      Topic:\n");
        lines.push_back("        Value: " + rviz_topic + '\n');
        lines.push_back("        Depth: 10\n");
        lines.push_back("      Name: " + rviz_name + '\n');
        lines.push_back("      Value: true\n");
    }

    while(std::getline(file_in, temp_str))
        if (temp_str.find("# General") != std::string::npos) {
            lines.push_back(temp_str + '\n');
            break;
        }

    while(std::getline(file_in, temp_str))
        lines.push_back(temp_str + '\n');

    file_in.close();

    std::ofstream file_out(rviz_file_path);
    if (!file_out)
        throw std::runtime_error("Error: Could not overwrite file: " + rviz_file_path);

    for (auto line: lines)
        file_out << line;

    file_out.close();
}


bool isSelectionValid(
        int selection1
        , int selection2
        , int selection3
        , int selection4)
{
    int selections[4]{ selection1, selection2, selection3, selection4 };
    for (int i{ 0 }; i < 4; ++i) {
        if (selections[i] == 0) // Skip if the network attribute is set to 'None'
            continue;
        for (int j{ i + 1 }; j < 4; ++j)
            if (selections[i] == selections[j])
                return false;
    }
    return true;
}


bool validateNetworkRanges(const NetworkParameterRanges& ranges)
{
    try {
        if (std::stod(ranges.delay_best) >= std::stod(ranges.delay_worst)
                or std::stod(ranges.jitter_best) >= std::stod(ranges.jitter_worst)
                or std::stod(ranges.rssi_best) <= std::stod(ranges.rssi_worst)
                or std::stod(ranges.packet_loss_best) >= std::stod(ranges.packet_loss_worst))
            return false;
    }
    catch (std::invalid_argument&) {
        return false;
    }
    return true;
}


std::string getNetworkAttributeRepresentative(int index)
{
    switch (index) {
    case 1:
        return "LINK_COLOUR_VALUE";
    case 2:
        return "LINK_OPACITY_VALUE";
    case 3:
        return "LINK_THICKNESS_VALUE";
    case 4:
        return "LINK_PACKET_DENSITY_VALUE";
    default:
        return "";
    }
}


std::string getNetworkAttribute(int index)
{
    switch (index) {
    case 0:
        return "DELAY";
    case 1:
        return "JITTER";
    case 2:
        return "RSSI";
    case 3:
        return "PACKET_LOSS";
    default:
        return "";
    }
}


void updateNetworkGeneralLaunch(
        int delay
        , int jitter
        , int rssi
        , int packet_loss
        , const NetworkParameterRanges& ranges)
{
    auto launch_file_path{ ament_index_cpp::get_package_share_directory("visually") + "/launch/" + "visualiser.launch.py"};
//    First, settting all attritbutes to DEFAULT
    editFile(launch_file_path, "LINK_COLOUR_VALUE =", "LINK_COLOUR_VALUE = DEFAULT");
    editFile(launch_file_path, "LINK_OPACITY_VALUE =", "LINK_OPACITY_VALUE = DEFAULT");
    editFile(launch_file_path, "LINK_THICKNESS_VALUE =", "LINK_THICKNESS_VALUE = DEFAULT");
    editFile(launch_file_path, "LINK_PACKET_DENSITY_VALUE =", "LINK_PACKET_DENSITY_VALUE = DEFAULT");

//    Then, setting the correct values
    if (delay)
        editFile(launch_file_path, getNetworkAttributeRepresentative(delay) + " = ", getNetworkAttributeRepresentative(delay) + " = DELAY");
    if (jitter)
        editFile(launch_file_path, getNetworkAttributeRepresentative(jitter) + " = ", getNetworkAttributeRepresentative(jitter) + " = JITTER");
    if (rssi)
        editFile(launch_file_path, getNetworkAttributeRepresentative(rssi) + " = ", getNetworkAttributeRepresentative(rssi) + " = RSSI");
    if (packet_loss)
        editFile(launch_file_path, getNetworkAttributeRepresentative(packet_loss) + " = ", getNetworkAttributeRepresentative(packet_loss) + " = PACKET_LOSS");

//    Then, setting the parameter ranges
    editFile(launch_file_path, "DELAY_BEST =", "DELAY_BEST = " + ranges.delay_best);
    editFile(launch_file_path, "DELAY_WORST =", "DELAY_WORST = " + ranges.delay_worst);
    editFile(launch_file_path, "JITTER_BEST =", "JITTER_BEST = " + ranges.jitter_best);
    editFile(launch_file_path, "JITTER_WORST =", "JITTER_WORST = " + ranges.jitter_worst);
    editFile(launch_file_path, "RSSI_BEST =", "RSSI_BEST = " + ranges.rssi_best);
    editFile(launch_file_path, "RSSI_WORST =", "RSSI_WORST = " + ranges.rssi_worst);
    editFile(launch_file_path, "PACKET_LOSS_BEST =", "PACKET_LOSS_BEST = " + ranges.packet_loss_best);
    editFile(launch_file_path, "PACKET_LOSS_WORST =", "PACKET_LOSS_WORST = " + ranges.packet_loss_worst);
}


void updateOfflineHeatmapTopicsRviz(
        const std::string& rsu_id
        , const std::string& obu_id
        , bool include)
{
    auto rviz_file_path{ ament_index_cpp::get_package_share_directory("visually") + "/rviz/" + "rviz_conf.rviz"};
    std::ifstream file_in(rviz_file_path);

    if (!file_in)
        throw std::runtime_error("Error: Could not open file for reading: " + rviz_file_path);

    std::vector<std::string> lines;

    std::string temp_str;
    while(std::getline(file_in, temp_str))
        if (temp_str.find("# Offline Heatmap") == std::string::npos)
            lines.push_back(temp_str + '\n');
        else
            break;
    lines.push_back(temp_str + '\n');

    if (include) {
        std::string rviz_topic {obu_id + '_' + rsu_id + "_offline_heatmap"};
        std::string rviz_name {obu_id + ':' + rsu_id + " Offline Heatmap"};

        lines.push_back("    - Class: rviz_default_plugins/MarkerArray\n");
        lines.push_back("      Enabled: true\n");
        lines.push_back("      Topic:\n");
        lines.push_back("        Value: " + rviz_topic + '\n');
        lines.push_back("        Depth: 10\n");
        lines.push_back("      Name: " + rviz_name + '\n');
        lines.push_back("      Value: true\n");
    }

    while(std::getline(file_in, temp_str))
        if (temp_str.find("# Offline Heatmap") != std::string::npos) {
            lines.push_back(temp_str + '\n');
            break;
        }

    while(std::getline(file_in, temp_str))
        lines.push_back(temp_str + '\n');

    file_in.close();

    std::ofstream file_out(rviz_file_path);
    if (!file_out)
        throw std::runtime_error("Error: Could not overwrite file: " + rviz_file_path);

    for (auto line: lines)
        file_out << line;

    file_out.close();
}


void updateOnlineHeatmapTopicsRviz(const std::vector<std::string>& topics, bool include)
{
    static const std::regex obu_cpmn_rgx{ "/(OBU_\\d+)/(RSU_\\d+)/cpmn" };

    auto rviz_file_path{ ament_index_cpp::get_package_share_directory("visually") + "/rviz/" + "rviz_conf.rviz"};
    std::ifstream file_in(rviz_file_path);

    if (!file_in)
        throw std::runtime_error("Error: Could not open file for reading: " + rviz_file_path);

    std::vector<std::string> lines;

    std::string temp_str;
    while(std::getline(file_in, temp_str))
        if (temp_str.find("# Online Heatmap") == std::string::npos)
            lines.push_back(temp_str + '\n');
        else
            break;
    lines.push_back(temp_str + '\n');

    std::string rviz_topic;
    std::string rviz_name;
    std::smatch match_results;

    if (include)
        for (const auto& topic: topics) {

            if (std::regex_search(topic, match_results, obu_cpmn_rgx)) {
                rviz_topic = match_results[1].str() + "_" + match_results[2].str() + "_cpmn";
                rviz_name = match_results[1].str() + ':' + match_results[2].str() + " Online Heatmap";

                for (const auto& attr: {"delay", "jitter", "rssi", "packet_loss"}) {
                    lines.push_back("    - Class: rviz_default_plugins/MarkerArray\n");
                    lines.push_back("      Enabled: true\n");
                    lines.push_back("      Topic:\n");
                    lines.push_back("        Value: " + rviz_topic + '_' + attr + '\n');
                    lines.push_back("        Depth: 10\n");
                    lines.push_back("      Name: " + rviz_name + ' ' + attr + '\n');
                    lines.push_back("      Value: true\n");
                }
            }
        }

    while(std::getline(file_in, temp_str))
        if (temp_str.find("# Online Heatmap") != std::string::npos) {
            lines.push_back(temp_str + '\n');
            break;
        }

    while(std::getline(file_in, temp_str))
        lines.push_back(temp_str + '\n');

    file_in.close();

    std::ofstream file_out(rviz_file_path);
    if (!file_out)
        throw std::runtime_error("Error: Could not overwrite file: " + rviz_file_path);

    for (auto line: lines)
        file_out << line;

    file_out.close();
}


void updateNetworkTargetedLaunch(
        const std::string& rsu_obu_con_dist_range
        , const std::string& target_rsu_id
        , const std::string& target_obu_id
        , const std::string& offline_heatmap_path
        , int offline_heatmap_attribute
        , bool display_real_time_heatmap
        , bool display_real_time_graphs)
{
    auto launch_file_path{ ament_index_cpp::get_package_share_directory("visually") + "/launch/" + "visualiser.launch.py"};

    editFile(launch_file_path, "RSU_OBU_CON_DIST = ", "RSU_OBU_CON_DIST = " + rsu_obu_con_dist_range);

    editFile(launch_file_path, "TARGET_RSU_ID = ", "TARGET_RSU_ID = \"" + target_rsu_id + '"');
    editFile(launch_file_path, "TARGET_OBU_ID = ", "TARGET_OBU_ID = \"" + target_obu_id + '"');

    editFile(launch_file_path, "OFF_HM_PATH = ", "OFF_HM_PATH = \"" + offline_heatmap_path + '"');
    editFile(launch_file_path, "OFF_HM_ATTR = ", "OFF_HM_ATTR = " + getNetworkAttribute(offline_heatmap_attribute));

    if (display_real_time_heatmap)
        editFile(launch_file_path, "ON_HM = ", "ON_HM = True");
    else
        editFile(launch_file_path, "ON_HM = ", "ON_HM = False");


    if (display_real_time_graphs)
        editFile(launch_file_path, "RT_GRAPHS = ", "RT_GRAPHS = True");
    else
        editFile(launch_file_path, "RT_GRAPHS = ", "RT_GRAPHS = False");
}


void updatePointcloudPathLaunch(
        const QString& pointcloud_path)
{
    auto launch_file_path{ ament_index_cpp::get_package_share_directory("visually") + "/launch/" + "visualiser.launch.py"};
    editFile(launch_file_path, "PCD_FILENAME = ", "PCD_FILENAME = " + ('"' + pointcloud_path.toStdString() + '"'));
}


void updatePointcloudTopicsRviz(bool include)
{
    auto rviz_file_path{ ament_index_cpp::get_package_share_directory("visually") + "/rviz/" + "rviz_conf.rviz"};
    std::ifstream file_in(rviz_file_path);

    if (!file_in)
        throw std::runtime_error("Error: Could not open file for reading: " + rviz_file_path);

    std::vector<std::string> lines;

    std::string temp_str;
    while(std::getline(file_in, temp_str))
        if (temp_str.find("# Pointcloud") == std::string::npos)
            lines.push_back(temp_str + '\n');
        else
            break;
    lines.push_back(temp_str + '\n');

    if (include) {
        lines.push_back("    - Alpha: 0.15\n");
        lines.push_back("      Class: rviz_default_plugins/PointCloud2\n");
        lines.push_back("      Color: 255; 255; 255\n");
        lines.push_back("      Size (Pixels): 1\n");
        lines.push_back("      Size (m): 0.009999999776482582\n");
        lines.push_back("      Style: Points\n");
        lines.push_back("      Topic:\n");
        lines.push_back("        Value: /pointcloud2\n");
        lines.push_back("      Enabled: true\n");
        lines.push_back("      Name: Pointcloud Data\n");
        lines.push_back("      Value: true\n");
    }

    while(std::getline(file_in, temp_str))
        if (temp_str.find("# Pointcloud") != std::string::npos) {
            lines.push_back(temp_str + '\n');
            break;
        }

    while(std::getline(file_in, temp_str))
        lines.push_back(temp_str + '\n');

    file_in.close();

    std::ofstream file_out(rviz_file_path);
    if (!file_out)
        throw std::runtime_error("Error: Could not overwrite file: " + rviz_file_path);

    for (auto line: lines)
        file_out << line;

    file_out.close();

}


void updateLaneletPathLaunch(
        const QString& lanelet_path)
{
    auto launch_file_path{ ament_index_cpp::get_package_share_directory("visually") + "/launch/" + "visualiser.launch.py"};
    editFile(launch_file_path, "LANELET_FILENAME = ", "LANELET_FILENAME = " + ('"' + lanelet_path.toStdString() + '"'));
}


void updateLaneletTopicsRviz(bool include)
{
    auto rviz_file_path{ ament_index_cpp::get_package_share_directory("visually") + "/rviz/" + "rviz_conf.rviz"};
    std::ifstream file_in(rviz_file_path);

    if (!file_in)
        throw std::runtime_error("Error: Could not open file for reading: " + rviz_file_path);

    std::vector<std::string> lines;

    std::string temp_str;
    while(std::getline(file_in, temp_str))
        if (temp_str.find("# Lanelet") == std::string::npos)
            lines.push_back(temp_str + '\n');
        else
            break;
    lines.push_back(temp_str + '\n');

    if (include) {
        lines.push_back("    - Class: rviz_default_plugins/MarkerArray\n");
        lines.push_back("      Enabled: true\n");
        lines.push_back("      Name: Lanelet2 Map\n");
        lines.push_back("      Topic:\n");
        lines.push_back("        Value: /vector_map_marker\n");
        lines.push_back("      Value: true\n");
    }

    while(std::getline(file_in, temp_str))
        if (temp_str.find("# Lanelet") != std::string::npos) {
            lines.push_back(temp_str + '\n');
            break;
        }

    while(std::getline(file_in, temp_str))
        lines.push_back(temp_str + '\n');

    file_in.close();

    std::ofstream file_out(rviz_file_path);
    if (!file_out)
        throw std::runtime_error("Error: Could not overwrite file: " + rviz_file_path);

    for (auto line: lines)
        file_out << line;

    file_out.close();

}


void updateMapOffsetLaunch(
        const QString& x
        , const QString& y
        , const QString& z)
{
    auto launch_file_path{ ament_index_cpp::get_package_share_directory("visually") + "/launch/" + "visualiser.launch.py"};
    editFile(launch_file_path, "MAP_OFFSET = ", "MAP_OFFSET = [" + x.toStdString() + ", " + y.toStdString() + ", " + z.toStdString() + "]");
}


void updateFocalRviz(
        const QString& x
        , const QString& y
        , const QString& z)
{
    auto rviz_file_path{ ament_index_cpp::get_package_share_directory("visually") + "/rviz/" + "rviz_conf.rviz"};
    std::ifstream file_in(rviz_file_path);

    if (!file_in)
        throw std::runtime_error("Error: Could not open file for reading: " + rviz_file_path);

    std::vector<std::string> lines;

    std::string temp_str;
    while(std::getline(file_in, temp_str)) {
        lines.push_back(temp_str + '\n');
        if (temp_str.find("Focal Point:") != std::string::npos) {
            for (int i = 0; i < 3; ++i)
                std::getline(file_in, temp_str);
            break;
        }
    }

    lines.push_back("        X: " + x.toStdString() + "\n");
    lines.push_back("        Y: " + y.toStdString() + "\n");
    lines.push_back("        Z: " + z.toStdString() + "\n");

    while(std::getline(file_in, temp_str))
        lines.push_back(temp_str + '\n');

    file_in.close();

    std::ofstream file_out(rviz_file_path);
    if (!file_out)
        throw std::runtime_error("Error: Could not overwrite file: " + rviz_file_path);

    for (auto line: lines)
        file_out << line;

    file_out.close();
}


} // namespace common
