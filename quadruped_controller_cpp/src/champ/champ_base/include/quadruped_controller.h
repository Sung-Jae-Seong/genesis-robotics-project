/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
Copyright (c) 2025, SJS

This file is licensed under the BSD-3-Clause License.
See the LICENSE file in the project root for the full license text.
*/

#ifndef QUADRUPED_CONTROLLER_H
#define QUADRUPED_CONTROLLER_H

#include <string>
#include <vector>
#include <array>
#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>
#include <thread>

#include <yaml_parser.h>

#include <champ/body_controller/body_controller.h>
#include <champ/utils/urdf_loader.h>
#include <champ/leg_controller/leg_controller.h>
#include <champ/kinematics/kinematics.h>

constexpr size_t NUM_JOINTS = 12;
constexpr size_t NUM_FEET = 4;

class QuadrupedController{
  public:
    QuadrupedController();
  
    void setURDFfromFile(std::string urdf_file_path);
    void setGaitConfig(const std::string& file_path);
    void setJointsMap(const std::string& file_path);
    void setLinksMap(const std::string& file_path);
    void setVelocityCommand(float linear_x, float linear_y, float linear_z, float angular_z);
    void setSpeedValue(float speed);
    void setTurnValue(float turn);
    float getSpeedValue() const;
    float getTurnValue() const;
    std::vector<std::string> getJointNames () const;
    std::array<float, NUM_JOINTS> getJointPositions();

    std::vector<std::string> joint_names_;
    std::vector<std::vector<std::string>> links_map;

    champ::Velocities req_vel_;
    champ::Pose req_pose_;
    champ::GaitConfig gait_config_;
    champ::QuadrupedBase base_;
    champ::BodyController body_controller_;
    champ::LegController leg_controller_;
    champ::Kinematics kinematics_;

    float speed;
    float turn;
    std::string urdf;
};

#endif  // QUADRUPED_CONTROLLER_H
