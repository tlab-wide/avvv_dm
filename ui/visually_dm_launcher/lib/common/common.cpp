#include "common.hpp"


namespace common
{


void configurePreprocessConfig(
        const QString& obu_tf_path,
        const QString& obu_freespace_path,
        const QString& obu_signal_path,
        const QString& obu_object_path,
        const QString& output_folder_path,
        const QString& obu_id)
{
    auto config_file_path{ ament_index_cpp::get_package_share_directory("csv2bag") + "/config/" + "config.yaml"};
    editFile(config_file_path, "input_obu_tf_path: ", "    input_obu_tf_path: \"" + obu_tf_path.toStdString() + "\"");
    editFile(config_file_path, "input_obu_object_info_path: ", "    input_obu_object_info_path: \"" + obu_object_path.toStdString() + "\"");
    editFile(config_file_path, "input_obu_signal_info_path: ", "    input_obu_signal_info_path: \"" + obu_signal_path.toStdString() + "\"");
    editFile(config_file_path, "input_obu_freespace_info_path: ", "    input_obu_freespace_info_path: \"" + obu_freespace_path.toStdString() + "\"");
    editFile(config_file_path, "output_folder_path: ", "    output_folder_path: \"" + output_folder_path.toStdString() + "/" + obu_id.toStdString() + "\"");
}


void setAnalyseConf(
        const std::string& input_rosbags
        , const std::string& input_csvs
        , const std::string& output_folder)
{
    auto confini_file_path{ std::string(std::getenv("AVVV_DM_HOME")) +
        "analyser/conf.ini" };
    editFile(confini_file_path, "csv_files_directory = "
             , std::string("csv_files_directory = ") + ('"' + input_csvs + '"'));
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


QStringList getRsuIds(
        const QStringList& rsus)
{
    QStringList rsu_ids;
    for (const QString rsu : rsus) {
        auto splitted{ rsu.split(" ") };
        rsu_ids << splitted.at(0);
    }
    return rsu_ids;
}


QStringList getObuIds(
        const QStringList& obus)
{
    QStringList obu_ids;
    for (const QString obu : obus) {
        auto splitted{ obu.split(" ") };
        obu_ids << splitted.at(0);
    }
    return obu_ids;
}


QStringList getCloudIds(
        const QStringList& clouds)
{
    QStringList cloud_ids;
    for (const QString cloud : clouds) {
        auto splitted{ cloud.split(" ") };
        cloud_ids << splitted.at(0);
    }
    return cloud_ids;
}


void refineTopics(
        const QStringList& rsus,
        const QStringList& obus,
        const std::vector<std::string>& topics,
        std::vector<std::string>& rsu_object_topics,
        std::vector<std::string>& rsu_freespace_topics,
        std::vector<std::string>& rsu_signal_topics,
        std::vector<std::string>& obu_object_topics,
        std::vector<std::string>& obu_freespace_topics,
        std::vector<std::string>& obu_signal_topics,
        std::vector<std::string>& obu_tf_topics,
        std::vector<std::string>& link_topics)
{
    auto rsu_id_rgx{ QRegularExpression("RSU_\\d+") };
    auto obu_id_rgx{ QRegularExpression("OBU_\\d+") };

    rsu_object_topics.clear();
    rsu_freespace_topics.clear();
    rsu_signal_topics.clear();
    obu_object_topics.clear();
    obu_freespace_topics.clear();
    obu_signal_topics.clear();
    obu_tf_topics.clear();
    link_topics.clear();

    for (const auto& topic : topics) {
        if (topic.find("RSU") != std::string::npos and
                topic.find("OBU") == std::string::npos and
                topic.find("network_status") == std::string::npos) {

            auto match{ rsu_id_rgx.match(QString::fromStdString(topic)) };

            if (topic.find("object_info") != std::string::npos) {
                if (rsus.contains(match.captured()))
                    rsu_object_topics.push_back(topic);
            }
            else if (topic.find("freespace_info") != std::string::npos) {
                if (rsus.contains(match.captured()))
                    rsu_freespace_topics.push_back(topic);
            }
            else if (topic.find("signal_info") != std::string::npos) {
                if (rsus.contains(match.captured()))
                    rsu_signal_topics.push_back(topic);
            }
        }
        else if (topic.find("object_infon") != std::string::npos) {
            auto rsu_match{ rsu_id_rgx.match(QString::fromStdString(topic)) };
            auto obu_match{ obu_id_rgx.match(QString::fromStdString(topic)) };

            if (rsus.contains(rsu_match.captured()) and
                    obus.contains(obu_match.captured()))
                obu_object_topics.push_back(topic);
        }
        else if (topic.find("freespace_infon") != std::string::npos) {
            auto rsu_match{ rsu_id_rgx.match(QString::fromStdString(topic)) };
            auto obu_match{ obu_id_rgx.match(QString::fromStdString(topic)) };

            if (rsus.contains(rsu_match.captured()) and
                    obus.contains(obu_match.captured()))
                obu_freespace_topics.push_back(topic);
        }
        else if (topic.find("signal_infon") != std::string::npos) {
            auto rsu_match{ rsu_id_rgx.match(QString::fromStdString(topic)) };
            auto obu_match{ obu_id_rgx.match(QString::fromStdString(topic)) };

            if (rsus.contains(rsu_match.captured()) and
                    obus.contains(obu_match.captured()))
                obu_signal_topics.push_back(topic);
        }
        else if (topic.find("tf") != std::string::npos) {
            auto match{ obu_id_rgx.match(QString::fromStdString(topic)) };

            if (obus.contains(match.captured()))
                obu_tf_topics.push_back(topic);
        }
        else if (topic.find("network_status") != std::string::npos) {
            auto rsu_match{ rsu_id_rgx.match(QString::fromStdString(topic)) };
            auto obu_match{ obu_id_rgx.match(QString::fromStdString(topic)) };

            if (rsus.contains(rsu_match.captured()) and
                    obus.contains(obu_match.captured()))
                link_topics.push_back(topic);
        }
    }
}


void updateLaunchConfigurations(
        const QStringList& rsus,
        const QStringList& obus,
        const QStringList& clds,
        const QStringList& sgnls,
        const std::vector<std::string>& rsu_object_topics,
        const std::vector<std::string>& rsu_freespace_topics,
        const std::vector<std::string>& rsu_signal_topics,
        const std::vector<std::string>& obu_object_topics,
        const std::vector<std::string>& obu_freespace_topics,
        const std::vector<std::string>& obu_signal_topics,
        const std::vector<std::string>& obu_tf_topics,
        const std::vector<std::string>& link_topics)
{
    auto launch_file_path{ ament_index_cpp::get_package_share_directory("visually_dm") + "/launch/" + "visualiser.launch.py"};
    std::ifstream file_in(launch_file_path);

    if (!file_in)
        throw std::runtime_error("Error: Could not open file for reading: " + launch_file_path);

    std::vector<std::string> lines;

    std::string temp_str;
    bool close_bracket;
    while(std::getline(file_in, temp_str)) {
        close_bracket = true;
        if (temp_str.find("RSU_LIST = [") != std::string::npos) {
            lines.push_back(temp_str + '\n');
            for (const auto& rsu : rsus)
                lines.push_back(std::string("\t\"") + rsu.toStdString() + "\",\n");
        }
        else if (temp_str.find("OBU_LIST = [") != std::string::npos) {
            lines.push_back(temp_str + '\n');
            for (const auto& obu : obus)
                lines.push_back(std::string("\t\"") + obu.toStdString() + "\",\n");
        }
        else if (temp_str.find("CLOUD = ") != std::string::npos) {
            if (clds.length() != 0)
                lines.push_back(std::string("CLOUD = \"") + clds.at(0).toStdString() + "\"\n");
            else
                lines.push_back(temp_str);
            close_bracket = false;
        }
        else if (temp_str.find("SIGNAL_LIST = [") != std::string::npos) {
            lines.push_back(temp_str + '\n');
            for (const auto& sgnl : sgnls)
                lines.push_back(std::string("\t\"") + sgnl.toStdString() + "\",\n");
        }
        else if (temp_str.find("RSU_DETECTION_TOPICS = [") != std::string::npos) {
            lines.push_back(temp_str + '\n');
            for (const auto& topic : rsu_object_topics)
                lines.push_back(std::string("\t\"") + topic + "\",\n");
        }
        else if (temp_str.find("RSU_FREESPACE_TOPICS = [") != std::string::npos) {
            lines.push_back(temp_str + '\n');
            for (const auto& topic : rsu_freespace_topics)
                lines.push_back(std::string("\t\"") + topic + "\",\n");
        }
        else if (temp_str.find("RSU_SIGNAL_TOPICS = [") != std::string::npos) {
            lines.push_back(temp_str + '\n');
            for (const auto& topic : rsu_signal_topics)
                lines.push_back(std::string("\t\"") + topic + "\",\n");
        }
        else if (temp_str.find("OBU_DETECTION_TOPICS = [") != std::string::npos) {
            lines.push_back(temp_str + '\n');
            for (const auto& topic : obu_object_topics)
                lines.push_back(std::string("\t\"") + topic + "\",\n");
        }
        else if (temp_str.find("OBU_FREESPACE_TOPICS = [") != std::string::npos) {
            lines.push_back(temp_str + '\n');
            for (const auto& topic : obu_freespace_topics)
                lines.push_back(std::string("\t\"") + topic + "\",\n");
        }
        else if (temp_str.find("OBU_SIGNAL_TOPICS = [") != std::string::npos) {
            lines.push_back(temp_str + '\n');
            for (const auto& topic : obu_signal_topics)
                lines.push_back(std::string("\t\"") + topic + "\",\n");
        }
        else if (temp_str.find("OBU_TOPICS = [") != std::string::npos) {
            lines.push_back(temp_str + '\n');
            for (const auto& topic : obu_tf_topics)
                lines.push_back(std::string("\t\"") + topic + "\",\n");
        }
        else if (temp_str.find("LINK_TOPICS = [") != std::string::npos) {
            lines.push_back(temp_str + '\n');
            for (const auto& topic : link_topics)
                lines.push_back(std::string("\t\"") + topic + "\",\n");
        }
        else {
            lines.push_back(temp_str + '\n');
            close_bracket = false;
        }

        // Discard the previous configurations of the current list
        if (close_bracket) {
            while(std::getline(file_in, temp_str) and
                    temp_str.find("]") == std::string::npos);
            lines.push_back(temp_str + '\n');
        }
    }

    file_in.close();

    std::ofstream file_out(launch_file_path);
    if (!file_out)
        throw std::runtime_error("Error: Could not overwrite file: " + launch_file_path);

    for (auto line: lines)
        file_out << line;

    file_out.close();
}


void getUniqueCrpIds(
        const QStringList& sgnls,
        std::set<std::string>& crp_ids)
{
    for (const auto& sgnl : sgnls)
        crp_ids.insert(sgnl.split(" ").at(1).toStdString());
}


void updateGeneralTopicsRviz(
        const QStringList& rsus,
        const QStringList& obus,
        const QStringList& clds,
        const std::set<std::string>& signal_crp_ids,
        const std::vector<std::string>& rsu_object_topics,
        const std::vector<std::string>& rsu_freespace_topics,
        const std::vector<std::string>& obu_object_topics,
        const std::vector<std::string>& obu_freespace_topics)
{
    auto rviz_file_path{ ament_index_cpp::get_package_share_directory("visually_dm") + "/rviz/" + "rviz_conf.rviz"};
    std::ifstream file_in(rviz_file_path);

    if (!file_in)
        throw std::runtime_error("Error: Could not open file for reading: " + rviz_file_path);

    auto rsu_id_rgx{ QRegularExpression("RSU_\\d+") };
    auto obu_id_rgx{ QRegularExpression("OBU_\\d+") };

    std::vector<std::string> lines;

    std::string temp_str;
    while(std::getline(file_in, temp_str)) {
        lines.push_back(temp_str + '\n');
        if (temp_str.find("# General") != std::string::npos)
            break;
    }

    for (const auto& entity : obus + rsus + clds) {
        lines.push_back("    - Class: rviz_default_plugins/MarkerArray\n");
        lines.push_back("      Enabled: true\n");
        lines.push_back("      Topic:\n");
        lines.push_back("        Value: /" + entity.toStdString() + '\n');
        lines.push_back("        Depth: 10\n");
        lines.push_back("      Name: " + entity.toStdString() + '\n');
        lines.push_back("      Value: true\n");
    }

    for (const auto& crp_id : signal_crp_ids) {
        lines.push_back("    - Class: rviz_default_plugins/MarkerArray\n");
        lines.push_back("      Enabled: true\n");
        lines.push_back("      Topic:\n");
        lines.push_back("        Value: /Intersection_" + crp_id + '\n');
        lines.push_back("        Depth: 10\n");
        lines.push_back("      Name: Intersection " + crp_id + '\n');
        lines.push_back("      Value: true\n");
    }

    for (const auto& topic: rsu_object_topics) {
        auto match{ rsu_id_rgx.match(QString::fromStdString(topic)) };
        auto rsu_id{ match.captured().toStdString() };

        lines.push_back("    - Class: rviz_default_plugins/MarkerArray\n");
        lines.push_back("      Enabled: true\n");
        lines.push_back("      Topic:\n");
        lines.push_back("        Value: " + topic + "/detections\n");
        lines.push_back("        Depth: 10\n");
        lines.push_back("      Name: " + rsu_id + " Detections" + '\n');
        lines.push_back("      Value: true\n");
    }

    for (const auto& topic: rsu_freespace_topics) {
        auto match{ rsu_id_rgx.match(QString::fromStdString(topic)) };
        auto rsu_id{ match.captured().toStdString() };

        lines.push_back("    - Class: rviz_default_plugins/MarkerArray\n");
        lines.push_back("      Enabled: true\n");
        lines.push_back("      Topic:\n");
        lines.push_back("        Value: " + topic + "/freespaces\n");
        lines.push_back("        Depth: 10\n");
        lines.push_back("      Name: " + rsu_id + " Freespaces" + '\n');
        lines.push_back("      Value: true\n");
    }

    for (const auto& topic: obu_object_topics) {
        auto obu_match{ obu_id_rgx.match(QString::fromStdString(topic)) };
        auto obu_id{ obu_match.captured().toStdString() };
        auto rsu_match{ rsu_id_rgx.match(QString::fromStdString(topic)) };
        auto rsu_id{ rsu_match.captured().toStdString() };

        lines.push_back("    - Class: rviz_default_plugins/MarkerArray\n");
        lines.push_back("      Enabled: true\n");
        lines.push_back("      Topic:\n");
        lines.push_back("        Value: " + topic + "/detections\n");
        lines.push_back("        Depth: 10\n");
        lines.push_back("      Name: " + obu_id + " Detections From " + rsu_id + '\n');
        lines.push_back("      Value: true\n");
    }

    for (const auto& topic: obu_freespace_topics) {
        auto obu_match{ obu_id_rgx.match(QString::fromStdString(topic)) };
        auto obu_id{ obu_match.captured().toStdString() };
        auto rsu_match{ rsu_id_rgx.match(QString::fromStdString(topic)) };
        auto rsu_id{ rsu_match.captured().toStdString() };

        lines.push_back("    - Class: rviz_default_plugins/MarkerArray\n");
        lines.push_back("      Enabled: true\n");
        lines.push_back("      Topic:\n");
        lines.push_back("        Value: " + topic + "/freespaces\n");
        lines.push_back("        Depth: 10\n");
        lines.push_back("      Name: " + obu_id + " Freespaces From " + rsu_id + '\n');
        lines.push_back("      Value: true\n");
    }

    for (const auto& rsu_id : rsus) {
        for (const auto& obu_id : obus) {
            lines.push_back("    - Class: rviz_default_plugins/MarkerArray\n");
            lines.push_back("      Enabled: true\n");
            lines.push_back("      Topic:\n");
            lines.push_back("        Value: /" + rsu_id.toStdString() + obu_id.toStdString() + "\n");
            lines.push_back("        Depth: 10\n");
            lines.push_back("      Name: " + rsu_id.toStdString() + "=" + obu_id.toStdString() + '\n');
            lines.push_back("      Value: true\n");
            lines.push_back("      Namespaces:\n");
            lines.push_back("        cld_obu_line_freespace_info: false\n");
            lines.push_back("        cld_obu_line_object_info: true\n");
            lines.push_back("        cld_obu_line_signal_info: false\n");
            lines.push_back("        cld_obu_spheres_freespace_info: false\n");
            lines.push_back("        cld_obu_spheres_object_info: true\n");
            lines.push_back("        cld_obu_spheres_signal_info: false\n");
            lines.push_back("        rsu_cld_line_freespace_info: false\n");
            lines.push_back("        rsu_cld_line_object_info: true\n");
            lines.push_back("        rsu_cld_line_signal_info: false\n");
            lines.push_back("        rsu_cld_spheres_freespace_info: false\n");
            lines.push_back("        rsu_cld_spheres_object_info: true\n");
            lines.push_back("        rsu_cld_spheres_signal_info: false\n");
            lines.push_back("        rsu_obu_line_freespace_info: false\n");
            lines.push_back("        rsu_obu_line_object_info: true\n");
            lines.push_back("        rsu_obu_line_signal_info: false\n");
            lines.push_back("        rsu_obu_spheres_freespace_info: false\n");
            lines.push_back("        rsu_obu_spheres_object_info: true\n");
            lines.push_back("        rsu_obu_spheres_signal_info: false\n");
        }
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


std::string getNetworkAttribute(int index)
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


void updateNetworkGeneralLaunch(
        int delay
        , int jitter
        , int rssi
        , int packet_loss
        , const NetworkParameterRanges& ranges)
{
    auto launch_file_path{ ament_index_cpp::get_package_share_directory("visually_dm") + "/launch/" + "visualiser.launch.py"};
    // Set all attritbutes to DEFAULT
    editFile(launch_file_path, "LINK_COLOUR_VALUE =", "LINK_COLOUR_VALUE = DEFAULT");
    editFile(launch_file_path, "LINK_OPACITY_VALUE =", "LINK_OPACITY_VALUE = DEFAULT");
    editFile(launch_file_path, "LINK_THICKNESS_VALUE =", "LINK_THICKNESS_VALUE = DEFAULT");
    editFile(launch_file_path, "LINK_PACKET_DENSITY_VALUE =", "LINK_PACKET_DENSITY_VALUE = DEFAULT");

    // Set the correct values
    if (delay)
        editFile(launch_file_path, getNetworkAttribute(delay) + " = ", getNetworkAttribute(delay) + " = DELAY");
    if (jitter)
        editFile(launch_file_path, getNetworkAttribute(jitter) + " = ", getNetworkAttribute(jitter) + " = JITTER");
    if (rssi)
        editFile(launch_file_path, getNetworkAttribute(rssi) + " = ", getNetworkAttribute(rssi) + " = RSSI");
    if (packet_loss)
        editFile(launch_file_path, getNetworkAttribute(packet_loss) + " = ", getNetworkAttribute(packet_loss) + " = PACKET_LOSS");

    // Set the parameter ranges
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
    auto rviz_file_path{ ament_index_cpp::get_package_share_directory("visually_dm") + "/rviz/" + "rviz_conf.rviz"};
    std::ifstream file_in(rviz_file_path);

    if (!file_in)
        throw std::runtime_error("Error: Could not open file for reading: " + rviz_file_path);

    std::vector<std::string> lines;

    std::string temp_str;
    while(std::getline(file_in, temp_str)) {
        lines.push_back(temp_str + '\n');
        if (temp_str.find("# Offline Heatmap") != std::string::npos)
            break;
    }

    if (include) {
        std::string rviz_topic {obu_id + '_' + rsu_id + "_offline_heatmap"};
        std::string rviz_name {obu_id + ':' + rsu_id + " Offline Heatmap"};

        lines.push_back("    - Class: rviz_default_plugins/MarkerArray\n");
        lines.push_back("      Enabled: true\n");
        lines.push_back("      Topic:\n");
        lines.push_back("        Value: /" + rviz_topic + '\n');
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


void updateOnlineHeatmapTopicsRviz(
        const std::vector<std::string>& obu_object_topics,
        const std::vector<std::string>& obu_freespace_topics,
        const std::vector<std::string>& obu_signal_topics,
        bool include)
{
    auto rviz_file_path{ ament_index_cpp::get_package_share_directory("visually_dm") + "/rviz/" + "rviz_conf.rviz"};
    std::ifstream file_in(rviz_file_path);

    if (!file_in)
        throw std::runtime_error("Error: Could not open file for reading: " + rviz_file_path);

    std::vector<std::string> lines;

    std::string temp_str;
    while(std::getline(file_in, temp_str)) {
        lines.push_back(temp_str + '\n');
        if (temp_str.find("# Online Heatmap") != std::string::npos)
            break;
    }

    std::vector<std::vector<std::string>> all_topics{
        obu_object_topics,
        obu_freespace_topics,
        obu_signal_topics};

    if (include)
        for (const auto& topics : all_topics)
            for (const auto& topic: topics)
                for (const auto& attr: {"delay", "jitter", "rssi", "packet_loss"}) {
                    lines.push_back("    - Class: rviz_default_plugins/MarkerArray\n");
                    lines.push_back("      Enabled: false\n");
                    lines.push_back("      Topic:\n");
                    lines.push_back("        Value: " + topic + "/heatmap/" + attr + '\n');
                    lines.push_back("        Depth: 10\n");
                    lines.push_back("      Name: " + topic.substr(1) + ' ' + attr + ' ' + "heatmap\n");
                    lines.push_back("      Value: true\n");
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
        , bool display_real_time_heatmap
        , bool display_real_time_graphs)
{
    auto launch_file_path{ ament_index_cpp::get_package_share_directory("visually_dm") + "/launch/" + "visualiser.launch.py"};

    editFile(launch_file_path, "RSU_OBU_CON_DIST = ", "RSU_OBU_CON_DIST = " + rsu_obu_con_dist_range);

    editFile(launch_file_path, "TARGET_RSU_ID = ", "TARGET_RSU_ID = \"" + target_rsu_id + '"');
    editFile(launch_file_path, "TARGET_OBU_ID = ", "TARGET_OBU_ID = \"" + target_obu_id + '"');

    std::string offline_heatmap_attribute;
    QFileInfo file_info(QString::fromStdString(offline_heatmap_path));
    if (file_info.completeBaseName().contains("Delay"))
        offline_heatmap_attribute = "DELAY";
    else
        offline_heatmap_attribute = "PACKET_LOSS";
    editFile(launch_file_path, "OFF_HM_PATH = ", "OFF_HM_PATH = \"" + offline_heatmap_path + '"');
    editFile(launch_file_path, "OFF_HM_ATTR = ", "OFF_HM_ATTR = " + offline_heatmap_attribute);

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
    auto launch_file_path{ ament_index_cpp::get_package_share_directory("visually_dm") + "/launch/" + "visualiser.launch.py"};
    editFile(launch_file_path, "PCD_FILENAME = ", "PCD_FILENAME = " + ('"' + pointcloud_path.toStdString() + '"'));
}


void updatePointcloudTopicsRviz(bool include)
{
    auto rviz_file_path{ ament_index_cpp::get_package_share_directory("visually_dm") + "/rviz/" + "rviz_conf.rviz"};
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
        lines.push_back("    - Alpha: 0.1\n");
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
    auto launch_file_path{ ament_index_cpp::get_package_share_directory("visually_dm") + "/launch/" + "visualiser.launch.py"};
    editFile(launch_file_path, "LANELET_FILENAME = ", "LANELET_FILENAME = " + ('"' + lanelet_path.toStdString() + '"'));
}


void updateLaneletTopicsRviz(bool include)
{
    auto rviz_file_path{ ament_index_cpp::get_package_share_directory("visually_dm") + "/rviz/" + "rviz_conf.rviz"};
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
    auto launch_file_path{ ament_index_cpp::get_package_share_directory("visually_dm") + "/launch/" + "visualiser.launch.py"};
    editFile(launch_file_path, "MAP_OFFSET = ", "MAP_OFFSET = [" + x.toStdString() + ", " + y.toStdString() + ", " + z.toStdString() + "]");
}


} // namespace common
