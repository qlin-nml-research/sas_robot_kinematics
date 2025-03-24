/*
# Copyright (c) 2020-2023 Murilo Marques Marinho
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
#include <sas_robot_kinematics/sas_robot_kinematics_client.hpp>
#include <sas_conversions/sas_conversions.hpp>
#include <sas_common/sas_common.hpp>
using std::placeholders::_1;

namespace sas
{

void RobotKinematicsClient::_callback_pose(const geometry_msgs::msg::PoseStamped &msg)
{
    pose_ = geometry_msgs_pose_to_dq(msg.pose);
    if(! enabled_)
    {
        enabled_ = true;
        //ROS_INFO_STREAM(ros::this_node::getName() + "::RobotKinematicsInterface enabled.");
        RCLCPP_INFO_STREAM(node_->get_logger(),"::"+get_class_name()+" enabled.");
    }
}

void RobotKinematicsClient::_callback_reference_frame(const geometry_msgs::msg::PoseStamped &msg)
{
    reference_frame_ = geometry_msgs_pose_stamped_to_dq(msg);
}

//#ifdef IS_SAS_PYTHON_BUILD
//RobotKinematicsInterface::RobotKinematicsInterface(const std::string &topic_prefix):
//    RobotKinematicsInterface(sas::common::get_static_node_handle(), topic_prefix)
//{
//    //Delegated to RobotKinematicsInterface::RobotKinematicsInterface(ros::NodeHandle &node_handle_publisher, ros::NodeHandle &node_handle_subscriber, const std::string &topic_prefix)
//}
//#endif

RobotKinematicsClient::RobotKinematicsClient(const std::shared_ptr<Node> &node, const std::string &topic_prefix):
    sas::Object("RobotKinematicsClient"),
    node_(node),
    enabled_(false),
    topic_prefix_(topic_prefix),
    pose_(0),
    reference_frame_(0)
{
    //ROS_INFO_STREAM(ros::this_node::getName() + "::Initializing RobotKinematicsInterface with prefix " + topic_prefix);
    RCLCPP_INFO_STREAM(node_->get_logger(),"::Initializing "+get_class_name()+" with prefix " + topic_prefix);

    //publisher_desired_pose_ = node_handle_publisher.advertise<geometry_msgs::PoseStamped>(topic_prefix + "/set/desired_pose", 1);
    publisher_desired_pose_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic_prefix + "/set/desired_pose",1);
    publisher_desired_pose_derivative_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic_prefix + "/set/desired_pose_derivative",1);
    //publisher_desired_interpolator_speed_ = node_handle_publisher.advertise<std_msgs::Float64>(topic_prefix + "/set/desired_interpolator_speed", 1);
    publisher_desired_interpolator_speed_ = node->create_publisher<sas_msgs::msg::Float64>(topic_prefix + "/set/desired_interpolator_speed",1);

    //subscriber_pose_ = node_handle_subscriber.subscribe(topic_prefix + "/get/pose", 1, &RobotKinematicsInterface::_callback_pose, this);
    subscriber_pose_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
                topic_prefix + "/get/pose", 1, std::bind(&RobotKinematicsClient::_callback_pose, this, _1)
                );
    //subscriber_reference_frame_ = node_handle_subscriber.subscribe(topic_prefix + "/get/reference_frame", 1, &RobotKinematicsInterface::_callback_reference_frame, this);
    subscriber_reference_frame_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
                topic_prefix + "/get/reference_frame", 1, std::bind(&RobotKinematicsClient::_callback_reference_frame, this, _1)
                );
}


bool RobotKinematicsClient::is_enabled() const
{
    return( is_unit(pose_) &&
            is_unit(reference_frame_));
}

DQ RobotKinematicsClient::get_pose() const
{
    if(is_enabled())
    {
        return pose_;
    }
    else
    {
        throw std::runtime_error("::Trying to get_desired_pose of an unitialized "+get_class_name());
    }
}

DQ RobotKinematicsClient::get_reference_frame() const
{
    if(is_enabled())
    {
        return reference_frame_;
    }
    else
        throw std::runtime_error("::"+get_class_name()+"::get_reference_frame()::trying to get reference frame but uninitialized.");
}

void RobotKinematicsClient::send_desired_pose(const DQ &desired_pose, const DQ& desired_pose_derivative) const
{
    publisher_desired_pose_->publish(dq_to_geometry_msgs_pose_stamped(desired_pose));
    if (desired_pose_derivative!=0)
    {
        const auto [t, t_dot, r, r_dot] = decompose_ff_pose_dot(desired_pose, desired_pose_derivative);
        geometry_msgs::msg::PoseStamped msg;
        msg.header = std_msgs::msg::Header();
        msg.pose.position = geometry_msgs::msg::Point();
        msg.pose.position.x=t_dot.q[1];
        msg.pose.position.y=t_dot.q[2];
        msg.pose.position.z=t_dot.q[3];
        msg.pose.orientation = geometry_msgs::msg::Quaternion();
        msg.pose.orientation.w=r_dot.q[0];
        msg.pose.orientation.x=r_dot.q[1];
        msg.pose.orientation.y=r_dot.q[2];
        msg.pose.orientation.z=r_dot.q[3];
        publisher_desired_pose_derivative_->publish(msg);
    }

}

void RobotKinematicsClient::send_desired_interpolator_speed(const double &interpolator_speed) const
{
    publisher_desired_interpolator_speed_->publish(double_to_std_msgs_float64(interpolator_speed));
}


/**
 * @brief Compose feedforward pose derivative term
 * @param t
 * @param t_dot
 * @param r
 * @param r_dot
 * @return x_dot
 */
DQ RobotKinematicsClient::compose_ff_pose_dot(const DQ& t, const DQ& t_dot, const DQ& r, const DQ& r_dot){
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
std::tuple<DQ, DQ, DQ, DQ> RobotKinematicsClient::decompose_ff_pose_dot(const DQ& x, const DQ& x_dot)
{
    const auto r = P(x);
    const auto t = translation(x);

    const auto r_dot = P(x_dot);
    const auto t_dot = 2 * (D(x_dot) - D(x) * conj(P(x)) * r_dot) * conj(P(x));
    return {t, t_dot, r, r_dot};
}



}


