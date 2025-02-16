/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
Copyright (c) 2025, SJS
This file is licensed under the BSD-3-Clause License.
See the LICENSE file in the project root for the full license text.
*/

#include <quadruped_controller.h>
#include <map>
#include <iostream>

champ::PhaseGenerator::Time stdTimeToChampTime(const std::chrono::steady_clock::time_point& time) {
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(time.time_since_epoch());
    return duration.count();
}

QuadrupedController::QuadrupedController():
    body_controller_(base_),
    leg_controller_(base_, stdTimeToChampTime(std::chrono::steady_clock::now())),
    kinematics_(base_)
{
    speed = 0.5;
    turn  = 1.0;
    urdf = getURDFfromFile("robot.urdf");

    std::string gait_file   = "./gait_config.yaml";
    std::string joints_file = "./joints_map.yaml";
    std::string links_file  = "./links_map.yaml";
    setGaitConfig(gait_file);
    setJointsMap(joints_file);
    setLinksMap(links_file);
}

std::vector<std::string> QuadrupedController::getJointNames() const {
    return joint_names_;
}

std::array<float, NUM_JOINTS> QuadrupedController::getJointPositions(){
    float target_joint_positions[NUM_JOINTS];
    geometry::Transformation target_foot_positions[NUM_FEET];

    body_controller_.poseCommand(target_foot_positions, req_pose_);

    auto current_time = std::chrono::steady_clock::now();
    leg_controller_.velocityCommand(target_foot_positions, req_vel_, stdTimeToChampTime(current_time));
    kinematics_.inverse(target_joint_positions, target_foot_positions);

    req_vel_.linear.x  = 1 * speed;
    req_vel_.linear.y  = 0 * speed;
    req_vel_.angular.z = 0 * turn;

    std::array<float, NUM_JOINTS> joint_arr;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        joint_arr[i] = target_joint_positions[i];
    }

    return joint_arr;
}

std::string QuadrupedController::getURDFfromFile(std::string urdf_path){
    std::string urdf_file_path = urdf_path;
    std::ifstream urdf_file(urdf_file_path);

    if (!urdf_file.is_open()) {
        std::cerr << "Failed to open URDF file: " << urdf_file_path << std::endl;
        return "";
    }

    std::stringstream urdf_buffer;
    urdf_buffer << urdf_file.rdbuf();
    std::string urdf = urdf_buffer.str();

    urdf_file.close();
    
    return urdf;
}

void QuadrupedController::setGaitConfig(const std::string& file_path) {
    try {
        auto gait_config = getGaitYaml(file_path);
        gait_config_.pantograph_leg         = std::get<bool>(gait_config["pantograph_leg"]);
        gait_config_.max_linear_velocity_x  = std::get<double>(gait_config["max_linear_velocity_x"]);
        gait_config_.max_linear_velocity_y  = std::get<double>(gait_config["max_linear_velocity_y"]);
        gait_config_.max_angular_velocity_z = std::get<double>(gait_config["max_angular_velocity_z"]);
        gait_config_.com_x_translation      = std::get<double>(gait_config["com_x_translation"]);
        gait_config_.swing_height           = std::get<double>(gait_config["swing_height"]);
        gait_config_.stance_depth           = std::get<double>(gait_config["stance_depth"]);
        gait_config_.stance_duration        = std::get<double>(gait_config["stance_duration"]);
        gait_config_.nominal_height         = std::get<double>(gait_config["nominal_height"]);
        gait_config_.knee_orientation       = ">>";
        base_.setGaitConfig(gait_config_);
        req_pose_.position.z = gait_config_.nominal_height;
    } catch (const std::bad_variant_access& e) {
        throw std::runtime_error("Gait config type error in " + file_path + ": " + e.what());
    } catch (const std::exception& e) {
        throw std::runtime_error("setGaitConfig error: " + std::string(e.what()));
    }
}

void QuadrupedController::setJointsMap(const std::string& file_path) {
    try {
        joint_names_ = getJointsYaml(file_path);
    } catch (const std::exception& e) {
        throw std::runtime_error("setJointNames error: " + std::string(e.what()));
    }
}

void QuadrupedController::setLinksMap(const std::string& file_path) {
    try {
        links_map = getLinksYaml(file_path);
        champ::URDF::loadFromString(base_, links_map, urdf);
    } catch (const std::exception& e) {
        throw std::runtime_error("setLinksMap error: " + std::string(e.what()));
    }
}

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <quadruped_controller.h>

namespace py = pybind11;

PYBIND11_MODULE(quadruped_controller_binding, m) {
    m.doc() = "Python bindings for quadruped_controller";
    
    py::class_<QuadrupedController>(m, "QuadrupedController")
        .def(py::init<>()) // 기본 생성자 바인딩
        .def("getJointNames", &QuadrupedController::getJointNames, 
        "Returns the joint names as a Python list of strings.")
        .def("getJointPositions", &QuadrupedController::getJointPositions, 
        "Returns the joint positions as a Python list of floats.");
}