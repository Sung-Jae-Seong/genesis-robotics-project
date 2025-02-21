/*
Copyright (c) 2025, SJS

This file is licensed under the BSD-3-Clause License.
See the LICENSE file in the project root for the full license text.
*/

#ifndef YAML_PARSER_H
#define YAML_PARSER_H

#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <map>
#include <iostream>
#include <variant>

std::map<std::string, std::variant<bool, double>> getGaitYaml(const std::string& file_path);
std::vector<std::string> getJointsYaml(const std::string& file_path);
std::vector<std::vector<std::string>> getLinksYaml(const std::string& file_path);

#endif  // YAML_PARSER_H