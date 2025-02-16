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
    std::string knee_orientation;
    std::string urdf = getURDFfromFile("robot.urdf");

    double loop_rate = 200.0;

    gait_config_.pantograph_leg = false;
    gait_config_.max_linear_velocity_x = 0.3;
    gait_config_.max_linear_velocity_y = 0.25;
    gait_config_.max_angular_velocity_z = 0.5;
    gait_config_.com_x_translation = 0.0;
    gait_config_.swing_height = 0.04;
    gait_config_.stance_depth = 0.01;
    gait_config_.stance_duration = 0.25;
    gait_config_.nominal_height = 0.225;
    knee_orientation = ">>";

    gait_config_.knee_orientation = knee_orientation.c_str();

    base_.setGaitConfig(gait_config_);

    std::vector<std::vector<std::string>> links_map = {
        {"lf_hip_link", "lf_upper_leg_link", "lf_lower_leg_link", "lf_foot_link"},
        {"rf_hip_link", "rf_upper_leg_link", "rf_lower_leg_link", "rf_foot_link"},
        {"lh_hip_link", "lh_upper_leg_link", "lh_lower_leg_link", "lh_foot_link"},
        {"rh_hip_link", "rh_upper_leg_link", "rh_lower_leg_link", "rh_foot_link"}
    };
    champ::URDF::loadFromString(base_, links_map, urdf);

    joint_names_ = {
        "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint", "lf_foot_joint",
        "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint", "rf_foot_joint",
        "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint", "lh_foot_joint",
        "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint", "rh_foot_joint"
    };
    

    req_pose_.position.z = gait_config_.nominal_height;
}

void QuadrupedController::controlLoop_() {
    float target_joint_positions[NUM_JOINTS];
    geometry::Transformation target_foot_positions[4];

    body_controller_.poseCommand(target_foot_positions, req_pose_);

    auto current_time = std::chrono::steady_clock::now();
    leg_controller_.velocityCommand(target_foot_positions, req_vel_, stdTimeToChampTime(current_time));
    kinematics_.inverse(target_joint_positions, target_foot_positions);

    req_vel_.linear.x = 1*speed;
    req_vel_.linear.y = 0*speed;
    req_vel_.angular.z = 0*turn;

    std::cout << joint_names_[0] << std::endl;
    std::cout << joint_names_[1] << std::endl;
    std::cout << joint_names_[2] << std::endl;
    std::cout << joint_names_[3] << std::endl;
    std::cout << joint_names_[4] << std::endl;
    std::cout << joint_names_[5] << std::endl;
    std::cout << joint_names_[6] << std::endl;
    std::cout << joint_names_[7] << std::endl;
    std::cout << joint_names_[8] << std::endl;
    std::cout << joint_names_[9] << std::endl;
    std::cout << joint_names_[10] << std::endl;
    std::cout << joint_names_[11] << std::endl;

    std::cout << target_joint_positions[0] << std::endl;
    std::cout << target_joint_positions[1] << std::endl;
    std::cout << target_joint_positions[2] << std::endl;
    std::cout << target_joint_positions[3] << std::endl;
    std::cout << target_joint_positions[4] << std::endl;
    std::cout << target_joint_positions[5] << std::endl;
    std::cout << target_joint_positions[6] << std::endl;
    std::cout << target_joint_positions[7] << std::endl;
    std::cout << target_joint_positions[9] << std::endl;
    std::cout << target_joint_positions[9] << std::endl;
    std::cout << target_joint_positions[10] << std::endl;
    std::cout << target_joint_positions[11] << std::endl;
}

std::vector<std::string> QuadrupedController::getJointNames() const {
    return joint_names_;
}

std::array<float, NUM_JOINTS> QuadrupedController::getJointPositions(){
    float target_joint_positions[NUM_JOINTS];
    geometry::Transformation target_foot_positions[4];

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

int main() {
    auto quadruped_controller = std::make_shared<QuadrupedController>();

    // 타이머 루프 실행
    double loop_rate_hz = 200.0; // 200Hz 루프
    auto period = std::chrono::milliseconds(static_cast<int>(1000 / loop_rate_hz));

    auto start_time = std::chrono::steady_clock::now();
    while (true) {
        std::this_thread::sleep_for(period); // 주기적 실행
        quadruped_controller->controlLoop_();

        auto now = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(now - start_time);

        if (elapsed_time.count() >= 5) {
            break; // 5초가 지나면 루프 종료
        }
    }

    return 0;
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