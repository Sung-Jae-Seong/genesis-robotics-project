/*
Copyright (c) 2025, SJS

This file is licensed under the BSD-3-Clause License.
See the LICENSE file in the project root for the full license text.
*/

#include "yaml_parser.h"

std::map<std::string, std::variant<bool, double>> getGaitYaml(const std::string& file_path) {
    std::map<std::string, std::variant<bool, double>> gait_config;
    YAML::Node config;

    try {
        config = YAML::LoadFile(file_path);
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading YAML file: " << e.what() << "\n";
        return gait_config;
    }

    if (config["gait_config"]) {
        for (const auto& it : config["gait_config"]) {
            std::string key = it.first.as<std::string>();

            if (it.second.IsScalar() && it.second.IsDefined()) {
                try {
                    gait_config[key] = it.second.as<double>();
                } catch (const YAML::TypedBadConversion<double>&) {
                    try {
                        gait_config[key] = it.second.as<bool>();
                    } catch (const YAML::Exception&) {
                        std::cerr << "Warning: Could not convert '" << key
                                  << "' to double or boolean. Skipping.\n";
                    }
                }
            }
        }
    }
    return gait_config;
}

std::vector<std::string> getJointsYaml(const std::string& file_path) {
    std::vector<std::string> joint_names;
    YAML::Node config;
    try {
        config = YAML::LoadFile(file_path);
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading YAML file: " << e.what() << "\n";
        return joint_names;
    }

    if (config["joints_map"]) {
        for (const auto& group : config["joints_map"]) {
            for (const auto& joint : group.second) {
                joint_names.push_back(joint.as<std::string>());
            }
        }
    }
    return joint_names;
}


std::vector<std::vector<std::string>> getLinksYaml(const std::string& file_path) {
    std::vector<std::vector<std::string>> links_map;
    YAML::Node config;
    try {
        config = YAML::LoadFile(file_path);
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading YAML file: " << e.what() << "\n";
        return links_map;
    }

    if (config["links_map"]) {
        for (const auto& group : config["links_map"]) {
            if (group.second.IsSequence()) {
                std::vector<std::string> link_group;
                for (const auto& link : group.second) {
                    link_group.push_back(link.as<std::string>());
                }
                links_map.push_back(link_group);
            }
        }
    }
    return links_map;
}
