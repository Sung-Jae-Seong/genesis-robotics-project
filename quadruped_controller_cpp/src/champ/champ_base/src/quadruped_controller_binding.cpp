#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <quadruped_controller.h>

namespace py = pybind11;

PYBIND11_MODULE(quadruped_controller_binding, m) {
    m.doc() = "Python bindings for quadruped_controller";

    // quadruped_controller.cpp의 함수들을 여기에 바인딩
    // m.def();
    
    // 클래스 바인딩 예제
    py::class_<QuadrupedController>(m, "QuadrupedController");
}
