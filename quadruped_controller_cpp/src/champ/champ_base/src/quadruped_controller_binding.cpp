#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <quadruped_controller.h>

namespace py = pybind11;

PYBIND11_MODULE(quadruped_controller_binding, m) {
    m.doc() = "Python bindings for quadruped_controller";
    
    // 클래스 바인딩 예제
    py::class_<QuadrupedController>(m, "QuadrupedController")
        .def(py::init<>()) // 기본 생성자 바인딩
        .def("getJointNames", [](QuadrupedController& self) {
            std::vector<std::string> joint_names = self.getJointNames();
            py::list py_joint_names;
            for (const auto& name : joint_names) {
                py_joint_names.append(name);
            }
            return py_joint_names;
        }, "Returns the joint names as a Python list of strings.")
        .def("getJointPositions", [](QuadrupedController& self) {
            std::array<float, 12> joint_positions = self.getJointPositions();
            py::list py_joint_positions;
            for (const auto& pos : joint_positions) {
                py_joint_positions.append(pos);
            }
            return py_joint_positions;
        }, "Returns the joint positions as a Python list of floats.");
}
