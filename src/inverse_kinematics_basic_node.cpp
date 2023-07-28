//~ Copyright (C) 2023 Łukasz Woliński
//~ You may use, distribute and modify this code under the terms of the BSD-3-Clause License.

#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>
#include "inverse_kinematics/inverse_kinematics.hpp"
#include "inverse_kinematics/helper_functions.hpp"
#include "rrlib_interfaces/msg/jacobian_stamped.hpp"
#include "rrlib_interfaces/msg/cartesian_trajectory_point.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace rrlib
{
class InverseKinematicsNode : public rclcpp::Node
{
public:
    InverseKinematicsNode();
    void SetParameters(const rclcpp::Parameter & p);
    void solveIK( const Eigen::VectorXd& pose_FK, const Eigen::MatrixXd& J, const Eigen::VectorXd& pose_des, const Eigen::VectorXd& vel_des, Eigen::VectorXd* q, Eigen::VectorXd* dqdt );
    
private:
    void JointStateCallback(const sensor_msgs::msg::JointState & msg);
    void JacobianStampedCallback(const rrlib_interfaces::msg::JacobianStamped & msg);
    void PoseStampedCallback(const geometry_msgs::msg::PoseStamped & msg);
    void CartesianTrajectoryCallback(const rrlib_interfaces::msg::CartesianTrajectoryPoint & msg);
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_state_;
    rclcpp::Subscription<rrlib_interfaces::msg::JacobianStamped>::SharedPtr subscription_jacobian_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_pose_;
    rclcpp::Subscription<rrlib_interfaces::msg::CartesianTrajectoryPoint>::SharedPtr subscription_cart_trajectory_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr publisher_joint_trajectory_;
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
    
    InverseKinematics inverse_kinematics_;
    Eigen::VectorXd q_msr_;
    Eigen::MatrixXd jacobian_;
    Eigen::VectorXd pose_;
    
};

InverseKinematicsNode::InverseKinematicsNode()
: Node("inverse_kinematics_basic")
{
    //initialize subscribers, publisher, etc.
    pose_.resize(7);
}

void InverseKinematicsNode::SetParameters(const rclcpp::Parameter & p)
{
    //~ if (dt <= 0.0 or clik_gain_pos < 0.0 or clik_gain_pos >= 2.0/dt or clik_gain_ori < 0.0 or clik_gain_ori >= 2.0/dt)
    //~ {
        //~ std::string message;
        //~ message = "Parameters are not set. Please, provide the correct values:\n->dt shall be greater than zero,\nclik_gain_pos and clik_gain_ori shall be greater than or equal to zero and smaller than 2/dt.\n";
        //~ std::cout << message;
    //~ }
    //~ else
    //~ {
        //~ inverse_kinematics_.SetParameters(dt, DOF, clik_gain_pos, clik_gain_ori);
    //~ }
}

void InverseKinematicsNode::JointStateCallback(const sensor_msgs::msg::JointState & msg)
{
    // get the joint state, save it to class member variable
    
    if (msg.position.size() < inverse_kinematics_.GetDOF())
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(),
            *this->get_clock(),
            5000,
            "To solve the inverse kinematics problem, there are %ld joint variables required, not %ld.",
            inverse_kinematics_.GetDOF(),
            msg.position.size());
    }
    else
    {
        // get the joint position vector "q" from the JointState type msg
        q_msr_ = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(msg.position.data(), inverse_kinematics_.GetDOF());
    }
}

void InverseKinematicsNode::JacobianStampedCallback(const rrlib_interfaces::msg::JacobianStamped & msg)
{
    // get the Jacobian, save it to class member variable
    if (msg.jacobian.jacobian_data.size() < (6 * inverse_kinematics_.GetDOF()) )
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(),
            *this->get_clock(),
            5000,
            "To solve the inverse kinematics problem, there are %ld joint variables required, not %ld.",
            inverse_kinematics_.GetDOF(),
            msg.jacobian.jacobian_data.size());
    }
    else
    {
        // get the Jacobian matrix "jacobian_" from the JacobianStamped type msg
        //~ jacobian_ = Eigen::Map<const Eigen::Matrix<double, 6, Eigen::Dynamic>, Eigen::Unaligned>(msg.jacobian.jacobian_data.data());
        jacobian_ = Eigen::Map<const Eigen::MatrixXd>(&msg.jacobian.jacobian_data[0], 6, inverse_kinematics_.GetDOF()); //TODO: check this
    }
}

void InverseKinematicsNode::PoseStampedCallback(const geometry_msgs::msg::PoseStamped & msg)
{
    pose_(0) = msg.pose.position.x;
    pose_(1) = msg.pose.position.y;
    pose_(2) = msg.pose.position.z;
    pose_(3) = msg.pose.orientation.x;
    pose_(4) = msg.pose.orientation.y;
    pose_(5) = msg.pose.orientation.z;
    pose_(6) = msg.pose.orientation.w;
}

void InverseKinematicsNode::CartesianTrajectoryCallback(const rrlib_interfaces::msg::CartesianTrajectoryPoint & msg)
{
    // get the desired end effector trajectory
    Eigen::VectorXd pose_des = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(msg.positions.data(), 7);
    Eigen::VectorXd vel_des = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(msg.velocities.data(), 7);
    
    // get the solution of the forward kinematics problem (end effector pose and manipulator's Jacobian)
    Eigen::VectorXd pose_FK = pose_;
    Eigen::MatrixXd J = jacobian_;
    // get the current measured joint position (new joint position is computed as: q += dqdt * dt
    Eigen::VectorXd q = q_msr_;
    // declare the joint velocity vector
    Eigen::VectorXd dqdt;
    
    // solve the inverse kinematics problem
    inverse_kinematics_.SolveIK(pose_FK, J, pose_des, vel_des, &q, &dqdt);
    
    // publish the solution
    trajectory_msgs::msg::JointTrajectoryPoint joint_trajectory_point;
    std::vector<double> positions(q.data(), q.data() + q.size());
    joint_trajectory_point.positions = positions;
    std::vector<double> velocities(dqdt.data(), dqdt.data() + dqdt.size());
    joint_trajectory_point.velocities = velocities;
    // TODO: maybe dqdt from the previous step also should be stored as a class member dqdt_prev_; then, the acceleration could be computed as:
    // d2qdt2 = (dqdt - dqdt_prev_) / dt;
    //~ std::vector<double> accelerations(d2qdt2.data(), d2qdt2.data() + d2qdt2.size());
    //~ joint_trajectory_point.accelerations = accelerations;
    joint_trajectory_point.time_from_start = msg.time_from_start;
    publisher_joint_trajectory_->publish(joint_trajectory_point);
}

} // namespace rrlib

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rrlib::InverseKinematicsNode>());
    rclcpp::shutdown();
    return 0;
}
