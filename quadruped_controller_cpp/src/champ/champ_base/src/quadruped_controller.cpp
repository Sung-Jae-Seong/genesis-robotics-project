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

    req_vel_.linear.x  = 0;
    req_vel_.linear.y  = 0;
    req_vel_.angular.z = 0;

    urdf.clear();
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

    std::array<float, NUM_JOINTS> joint_arr;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        joint_arr[i] = target_joint_positions[i];
    }

    return joint_arr;
}

void QuadrupedController::setURDFfromFile(std::string urdf_path){
    std::string urdf_file_path = urdf_path;
    std::ifstream urdf_file(urdf_file_path);

    if (!urdf_file.is_open()) {
        std::cerr << "Failed to open URDF file: " << urdf_file_path << std::endl;
    }

    std::stringstream urdf_buffer;
    urdf_buffer << urdf_file.rdbuf();
    urdf = urdf_buffer.str();

    urdf_file.close();
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

void QuadrupedController::setVelocityCommand(float linear_x, float linear_y, float linear_z, float angular_z) {
    req_vel_.linear.x  = linear_x * speed;
    req_vel_.linear.y  = linear_y * speed;
    req_vel_.linear.z  = linear_z * speed;
    req_vel_.angular.z = angular_z * turn;
}

void QuadrupedController::setSpeedValue(float new_speed) {
    speed = new_speed;
}

void QuadrupedController::setTurnValue(float new_turn) {
    turn = new_turn;
}

float QuadrupedController::getSpeedValue() const {
    return speed;
}
float QuadrupedController::getTurnValue() const {
    return turn;
}

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <quadruped_controller.h>

namespace py = pybind11;

PYBIND11_MODULE(quadruped_controller_binding, m) {
    m.doc() = "Python bindings for quadruped_controller";
    
    py::class_<QuadrupedController>(m, "QuadrupedController")
        .def(py::init<>()) // 기본 생성자 바인딩
        .def("setURDFfromFile", &QuadrupedController::setURDFfromFile,        // setURDFfromFile
            "Reads URDF from the specified file and stores it internally.")
       .def("setGaitConfig", &QuadrupedController::setGaitConfig,             // setGaitConfig
            "Loads gait config from a YAML file.")
       .def("setJointsMap", &QuadrupedController::setJointsMap,               // setJointsMap
            "Loads joint names from a YAML file.")
       .def("setLinksMap", &QuadrupedController::setLinksMap,                 // setLinksMap
            "Loads link descriptions from a YAML file and applies them to the URDF base.")
        .def("getJointNames", &QuadrupedController::getJointNames,            // getJointNames
        "Returns the joint names as a Python list of strings.")
        .def("getJointPositions", &QuadrupedController::getJointPositions,    // getJointPositions
        "Returns the joint positions as a Python list of floats.")
        .def("setVelocityCommand", &QuadrupedController::setVelocityCommand,  // setVelocityCommand
            "Sets the velocity command for the quadruped.",
            py::arg("linear_x"), py::arg("linear_y"), py::arg("linear_z"), py::arg("angular_z"))
        .def("setSpeedValue", &QuadrupedController::setSpeedValue,            // setSpeedValue
            "Sets the base speed value for movement.",
            py::arg("speed"))
        .def("setTurnValue", &QuadrupedController::setTurnValue,              // setTurnValue
            "Sets the base turn rate value.",
            py::arg("turn"))
        .def("getSpeedValue", &QuadrupedController::getSpeedValue)            // getSpeedValue
        .def("getTurnValue", &QuadrupedController::getTurnValue);             // getTurnValue
}