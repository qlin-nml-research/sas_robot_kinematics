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

#include <rclcpp/rclcpp.hpp>

#include <sas_robot_kinematics/sas_robot_kinematics_client.hpp>
#include <sas_robot_kinematics/sas_robot_kinematics_server.hpp>

namespace py = pybind11;
using RKC = sas::RobotKinematicsClient;
using RKS = sas::RobotKinematicsServer;

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


    py::class_<RKC>(m, "RobotKinematicsClient")
            .def(py::init<const std::shared_ptr<rclcpp::Node>&,const std::string&>())
            .def("is_enabled",&RKC::is_enabled)
            .def("get_pose",[get_py_dq_from_cpp](const RKC &self)->py::object {
                return get_py_dq_from_cpp(self.get_pose());
            })
            .def("get_reference_frame",[get_py_dq_from_cpp](const RKC &self)->py::object {
                return get_py_dq_from_cpp(self.get_reference_frame());
            })
            .def("send_desired_pose",[get_cpp_dq_from_py](RKC &self, const py::object& pose_py, const py::object& pose_derv_py) {
                self.send_desired_pose(get_cpp_dq_from_py(pose_py), get_cpp_dq_from_py(pose_derv_py));
                }, py::arg("desired_pose"), py::arg("desired_pose_derivative")=dq_py_class(VectorXd::Zero(8)))
            .def("send_desired_interpolator_speed",&RKC::send_desired_interpolator_speed);

    py::class_<RKS>(m, "RobotKinematicsServer")
            .def(py::init<const std::shared_ptr<rclcpp::Node>&,const std::string&>())
            .def("is_enabled",&RKS::is_enabled)
            .def("get_desired_pose",[get_py_dq_from_cpp](const RKS &self)->py::object {
                return get_py_dq_from_cpp(self.get_desired_pose());
            })
            .def("get_desired_pose_derivative",[get_py_dq_from_cpp](const RKS &self)->py::object {
                return get_py_dq_from_cpp(self.get_desired_pose_derivative());
            })
            .def("get_desired_interpolator_speed",&RKS::get_desired_interpolator_speed)
            .def("send_pose",[get_cpp_dq_from_py](RKS &self, const py::object& pose_py) {
                self.send_pose(get_cpp_dq_from_py(pose_py));
                }, py::arg("current_pose"))
            .def("send_reference_frame",[get_cpp_dq_from_py](RKS &self, const py::object& pose_py) {
                self.send_reference_frame(get_cpp_dq_from_py(pose_py));
                }, py::arg("reference_frame"));

//    py::class_<RKC>(m, "RobotKinematicsClient")
//            .def(py::init<const std::shared_ptr<rclcpp::Node>&,const std::string&>())
//            .def("is_enabled",&RKC::is_enabled)
//            .def("get_pose",&RKC::get_pose)
//            .def("get_reference_frame",&RKC::get_reference_frame)
//            .def("send_desired_pose",&RKC::send_desired_pose, py::arg("desired_pose"), py::arg("desired_pose_derivative")=DQ(0))
//            .def("send_desired_interpolator_speed",&RKC::send_desired_interpolator_speed);
//
//    py::class_<RKS>(m, "RobotKinematicsServer")
//            .def(py::init<const std::shared_ptr<rclcpp::Node>&,const std::string&>())
//            .def("get_desired_pose",&RKS::get_desired_pose)
//            .def("get_desired_interpolator_speed",&RKS::get_desired_interpolator_speed)
//            .def("is_enabled",&RKS::is_enabled)
//            .def("send_pose",&RKS::send_pose)
//            .def("send_reference_frame",&RKS::send_reference_frame);

    m.def("compose_pose_dot", &RKS::compose_ff_pose_dot, "Compose feedforward pose derivative term");
    m.def("decompose_pose_dot", &RKS::decompose_ff_pose_dot, "Decompose feedforward pose derivative term");

}
