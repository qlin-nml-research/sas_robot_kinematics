/*
# Copyright (c) 2016-2022 Murilo Marques Marinho
#
#    This file is part of sas_robot_driver.
#
#    sas_robot_driver is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_driver.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# ################################################################*/
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <sas_robot_kinematics/sas_robot_kinematics_interface.h>
#include <sas_robot_kinematics/sas_robot_kinematics_provider.h>

namespace py = pybind11;
using RKI = sas::RobotKinematicsInterface;
using RKP = sas::RobotKinematicsProvider;

PYBIND11_MODULE(_sas_robot_kinematics, m) {

    py::object dq_py_class = py::module_::import("dqrobotics").attr("DQ");
    auto get_py_dq_from_cpp = [dq_py_class](const DQ &dq_cpp)->py::object
    {
        return dq_py_class(dq_cpp.vec8());
    };

    auto get_cpp_dq_from_py = [dq_py_class](const py::object &pose_py)->DQ {
        if (!py::isinstance(pose_py, dq_py_class)) {
            throw std::runtime_error("Expected a dqrobotics.DQ object");
        }
        const VectorXd pose_vec = dq_py_class.attr("vec8")(pose_py).cast<VectorXd>();
        return DQ(pose_vec);
    };


    py::class_<RKI>(m, "RobotKinematicsInterface")
            .def(py::init<const std::string&>())
            .def("is_enabled",&RKI::is_enabled)
            .def("get_pose",[get_py_dq_from_cpp](const RKI &self)->py::object {
                return get_py_dq_from_cpp(self.get_pose());
            })
            .def("get_reference_frame",[get_py_dq_from_cpp](const RKI &self)->py::object {
                return get_py_dq_from_cpp(self.get_reference_frame());
            })
            .def("send_desired_pose",[get_cpp_dq_from_py](RKI &self, const py::object& pose_py) {
                self.send_desired_pose(get_cpp_dq_from_py(pose_py));
                }, py::arg("desired_pose"))
            .def("send_desired_interpolator_speed",&RKI::send_desired_interpolator_speed);
            
    py::class_<RKP>(m, "RobotKinematicsProvider")
            .def(py::init<const std::string&>())
            .def("get_desired_pose",[get_py_dq_from_cpp](const RKP &self)->py::object {
                return get_py_dq_from_cpp(self.get_desired_pose());
            })
            .def("get_desired_interpolator_speed",&RKP::get_desired_interpolator_speed)
            .def("send_pose",[get_cpp_dq_from_py](RKP &self, const py::object& pose_py) {
                self.send_pose(get_cpp_dq_from_py(pose_py));
                }, py::arg("current_pose"))
            .def("send_reference_frame",[get_cpp_dq_from_py](RKP &self, const py::object& pose_py) {
                self.send_reference_frame(get_cpp_dq_from_py(pose_py));
                }, py::arg("reference_frame"));

}
