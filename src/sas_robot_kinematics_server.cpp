/*
# Copyright (c) 2020-2022 Murilo Marques Marinho
#
#    This file is part of sas_robot_kinematics.
#
#    sas_robot_kinematics is free software: you can redistribute it and/or modify
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
#    along with sas_robot_kinematics.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# ################################################################*/
#include <sas_robot_kinematics/sas_robot_kinematics_server.hpp>
#include <sas_conversions/sas_conversions.hpp>
#include <sas_common/sas_common.hpp>
using std::placeholders::_1;

namespace sas
{

void RobotKinematicsServer::_callback_desired_pose(const geometry_msgs::msg::PoseStamped &msg)
{
    desired_pose_ = geometry_msgs_pose_to_dq(msg.pose);
}

void RobotKinematicsServer::_callback_desired_pose_derivative(const geometry_msgs::msg::PoseStamped &msg)
{
    if (!is_unit(desired_pose_))
    {
        // throw std::runtime_error("::Trying to set desired_pose_derivative without setting desired_pose first.");
        return; // Ignore the message
    }
    const DQ t_dot = DQ(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    const DQ r_dot = DQ(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
    desired_pose_derivative_ = compose_ff_pose_dot(translation(desired_pose_), t_dot, rotation(desired_pose_), r_dot);
}

void RobotKinematicsServer::_callback_desired_interpolator_speed(const sas_msgs::msg::Float64 &msg)
{
    desired_interpolator_speed_ = std_msgs_float64_to_double(msg);
}

//#ifdef IS_SAS_PYTHON_BUILD
//RobotKinematicsProvider::RobotKinematicsProvider(const std::string& topic_prefix):
//    RobotKinematicsProvider(sas::common::get_static_node_handle(),topic_prefix)
//{
//    //Delegated
//}
//#endif

RobotKinematicsServer::RobotKinematicsServer(const std::shared_ptr<Node> &node, const std::string &topic_prefix):
    sas::Object("RobotKinematicsServer"),
    node_(node),
    enabled_(false),
    topic_prefix_(topic_prefix),
    desired_pose_(0),
    desired_pose_derivative_(0)
{
    //ROS_INFO_STREAM(ros::this_node::getName() + "::Initializing RobotKinematicsProvider with prefix " + topic_prefix);
    RCLCPP_INFO_STREAM(node_->get_logger(),"::Initializing "+get_class_name()+" with prefix " + topic_prefix);

    //publisher_pose_ = nodehandle_publisher.advertise<geometry_msgs::PoseStamped>(topic_prefix + "/get/pose", 1);
    publisher_pose_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic_prefix + "/get/pose",1);
    //publisher_reference_frame_ = nodehandle_publisher.advertise<geometry_msgs::PoseStamped>(topic_prefix + "/get/reference_frame", 1);
    publisher_reference_frame_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic_prefix + "/get/reference_frame",1);

    //subscriber_desired_pose_ = nodehandle_subscriber.subscribe(topic_prefix + "/set/desired_pose", 1, &RobotKinematicsProvider::_callback_desired_pose, this);
    subscriber_desired_pose_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
                topic_prefix + "/set/desired_pose", 1, std::bind(&RobotKinematicsServer::_callback_desired_pose, this, _1)
                );
    subscriber_desired_pose_derivative_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
            topic_prefix + "/set/desired_pose_derivative", 1, std::bind(&RobotKinematicsServer::_callback_desired_pose_derivative, this, _1)
            );
    //subscriber_desired_interpolator_speed_ = nodehandle_subscriber.subscribe(topic_prefix + "/set/desired_interpolator_speed", 1, &RobotKinematicsProvider::_callback_desired_interpolator_speed, this);
    subscriber_desired_interpolator_speed_ = node->create_subscription<sas_msgs::msg::Float64>(
                topic_prefix + "/set/desired_interpolator_speed", 1, std::bind(&RobotKinematicsServer::_callback_desired_interpolator_speed, this, _1)
                );
}

bool RobotKinematicsServer::is_enabled() const
{
    return is_unit(desired_pose_);
}

DQ RobotKinematicsServer::get_desired_pose() const
{
    if(is_enabled())
    {
        return desired_pose_;
    }
    else
    {
        throw std::runtime_error("::Trying to get_desired_pose of an unitialized "+get_class_name()+"");
    }
}

DQ RobotKinematicsServer::get_desired_pose_derivative() const
{
    if(is_enabled() || desired_pose_derivative_ != 0)
    {
        return desired_pose_derivative_;
    }
    else
    {
        throw std::runtime_error("::Trying to get_desired_pose_derivative of an unitialized "+get_class_name()+" or without setting desired_pose first.");
    }

}


double RobotKinematicsServer::get_desired_interpolator_speed() const
{
    return desired_interpolator_speed_;
}

void RobotKinematicsServer::send_pose(const DQ &pose) const
{
    publisher_pose_->publish(dq_to_geometry_msgs_pose_stamped(pose));
}

void RobotKinematicsServer::send_reference_frame(const DQ &reference_frame) const
{
    publisher_reference_frame_->publish(sas::dq_to_geometry_msgs_pose_stamped(reference_frame));
}


/**
 * @brief Compose feedforward pose derivative term
 * @param t
 * @param t_dot
 * @param r
 * @param r_dot
 * @return x_dot
 */
DQ RobotKinematicsServer::compose_ff_pose_dot(const DQ& t, const DQ& t_dot, const DQ& r, const DQ& r_dot){
    const auto x = r + 0.5 * E_ * t * r;
    const auto x_dot = r_dot + 0.5 * E_ * (t_dot * r + t * r_dot);
    return x_dot;
}

/**
 * @brief Decompose feedforward pose and pose derivative into translation and rotation
 * @param x
 * @param x_dot
 * @return
 */
std::tuple<DQ, DQ, DQ, DQ> RobotKinematicsServer::decompose_ff_pose_dot(const DQ& x, const DQ& x_dot)
{
    const auto r = P(x);
    const auto t = translation(x);

    const auto r_dot = P(x_dot);
    const auto t_dot = 2 * (D(x_dot) - D(x) * conj(P(x)) * r_dot) * conj(P(x));
    return {t, t_dot, r, r_dot};
}


}


