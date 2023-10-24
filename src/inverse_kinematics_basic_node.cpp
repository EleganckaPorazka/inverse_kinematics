//~ Copyright (C) 2023 Łukasz Woliński
//~ You may use, distribute and modify this code under the terms of the BSD-3-Clause License.

#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>
#include "inverse_kinematics/inverse_kinematics_basic.hpp"
#include "inverse_kinematics/helper_functions.hpp"
#include "rrlib_interfaces/msg/jacobian_stamped.hpp"
#include "rrlib_interfaces/msg/cartesian_trajectory_point.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace rrlib
{
class InverseKinematicsNode : public rclcpp::Node
{
public:
    InverseKinematicsNode();
    void solveIK( const Eigen::VectorXd& pose_FK, const Eigen::MatrixXd& J, const Eigen::VectorXd& pose_des, const Eigen::VectorXd& vel_des, Eigen::VectorXd* q, Eigen::VectorXd* dqdt );
    
private:
    rcl_interfaces::msg::SetParametersResult ParametersCallback(const std::vector<rclcpp::Parameter> &parameters);
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    void JointStateCallback(const sensor_msgs::msg::JointState & msg);
    void PoseStampedCallback(const geometry_msgs::msg::PoseStamped & msg);
    void JacobianStampedCallback(const rrlib_interfaces::msg::JacobianStamped & msg);
    void CartesianTrajectoryCallback(const rrlib_interfaces::msg::CartesianTrajectoryPoint & msg);
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_state_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_pose_;
    rclcpp::Subscription<rrlib_interfaces::msg::JacobianStamped>::SharedPtr subscription_jacobian_;
    rclcpp::Subscription<rrlib_interfaces::msg::CartesianTrajectoryPoint>::SharedPtr subscription_cart_trajectory_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr publisher_joint_trajectory_;
    
    double dt_;     // time step (in seconds)
    size_t DOF_;    // number of degrees of freedom of the manipulator
    bool PARAMETERS_OK_;  // a flag to be set to true if the parameters are viable for the inverse kinematics computation, and false otherwise
    InverseKinematics inverse_kinematics_;
    Eigen::VectorXd pose_;      // the end effector pose from the forward kinematics solver (position and orientation, 7 elements)
    Eigen::MatrixXd jacobian_;  // the Jacobian from the forward kinematics solver (size: 6xDOF_)
    Eigen::VectorXd q_msr_;     // measured joint positions (in radians, size: DOF_)
};

InverseKinematicsNode::InverseKinematicsNode()
: Node("inverse_kinematics_basic")
{
    //initialize subscribers, publisher, parameters, etc.
    using std::placeholders::_1;
    
    RCLCPP_INFO(this->get_logger(), "Starting the node.");
    
    // initialize class members that better be initialized
    dt_ = 0.0;
    DOF_ = 1; // should be 0, but what if the node starts without setting the parameters and then tries to read the Jacobian into a 6x0 matrix? 
    PARAMETERS_OK_ = false;
    pose_.resize(7);
    
    // declare parameters and create the callback for handling their changes
    this->declare_parameter("DOF", 0);
    this->declare_parameter("dt", 0.0);
    std::vector<double> clik_gains = {0.0, 0.0};
    this->declare_parameter("clik_gains", clik_gains);
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&InverseKinematicsNode::ParametersCallback, this, _1));
    
    // call explicitly the ParametersCallback to handle the parameter changes from the launcher
    std::vector<std::string> param_names = {"DOF", "dt", "clik_gains"};
    std::vector<rclcpp::Parameter> parameters = this->get_parameters(param_names);
    rcl_interfaces::msg::SetParametersResult set_param_result = InverseKinematicsNode::ParametersCallback(parameters);
    
    // create a joint_states subscriber to get the joint positions
    subscription_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states",
        10,
        std::bind(&InverseKinematicsNode::JointStateCallback,
        this,
        _1));
    // create a subscription for the end effector pose
    subscription_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("fwd_kin_pose",
        10,
        std::bind(&InverseKinematicsNode::PoseStampedCallback,
        this,
        _1));
    // create a subscription for the Jacobian
    subscription_jacobian_ = this->create_subscription<rrlib_interfaces::msg::JacobianStamped>("fwd_kin_jacobian",
        10,
        std::bind(&InverseKinematicsNode::JacobianStampedCallback,
        this,
        _1));
    // create a subscription for the desired end effector pose and velocity
    subscription_cart_trajectory_ = this->create_subscription<rrlib_interfaces::msg::CartesianTrajectoryPoint>("cart_sin_traj",
        10,
        std::bind(&InverseKinematicsNode::CartesianTrajectoryCallback,
        this,
        _1));
    // create a publisher for the joint trajectory point
    publisher_joint_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>("ik_jnt_traj", 10);
}

rcl_interfaces::msg::SetParametersResult InverseKinematicsNode::ParametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
    // Helpful manuals and tutorials for this:
    // https://docs.ros.org/en/humble/Tutorials/Intermediate/Monitoring-For-Parameter-Changes-CPP.html
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    // https://roboticsbackend.com/ros2-rclcpp-parameter-callback/
    
    RCLCPP_INFO(this->get_logger(), "Received a parameter update.");
    
    rcl_interfaces::msg::SetParametersResult result;
    //TODO: do something with result.successful and result.reason
    result.successful = false;
    result.reason = "";
    //TODO: do it better:
    bool result_DOF = false;
    bool result_dt = false;
    bool result_clik_gains = false;
    
    for (const auto &param : parameters)
    {
        if (param.get_name() == "DOF")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                if (param.as_int() > 0)
                {
                    DOF_ = param.as_int();
                    inverse_kinematics_.SetDOF(DOF_);
                    result.successful = true;
                    result_DOF = true;
                    RCLCPP_INFO(this->get_logger(), "Parameter \"%s\" updated.", param.get_name().c_str());
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "The parameter \"%s\" shall be greater than 0.", param.get_name().c_str());
                    result.successful = false;
                    result.reason = "The number of degrees of freedom (DOF) shall be greater than 0.";
                    result_DOF = false;
                }
            }
        }
        if (param.get_name() == "dt")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                if (param.as_double() > 0.0)
                {
                    dt_ = param.as_double();
                    inverse_kinematics_.SetTimeStep(dt_);
                    result.successful = true;
                    result_dt = true;
                    RCLCPP_INFO(this->get_logger(), "Parameter \"%s\" updated.", param.get_name().c_str());
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "The parameter \"%s\" shall be greater than 0.0.", param.get_name().c_str());
                    result.successful = false;
                    result.reason = "Time step dt has to be greater than 0.0.";
                    result_dt = false;
                }
            }
        }
        if (param.get_name() == "clik_gains")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
            {
                if (param.as_double_array().size() == 2)
                {
                    if (dt_ > 0.0)
                    {
                        if (param.as_double_array()[0] >= 0.0
                            and param.as_double_array()[0] < (2.0 / dt_)
                            and param.as_double_array()[1] >= 0.0
                            and param.as_double_array()[1] < (2.0 / dt_))
                        {
                            inverse_kinematics_.SetCLIKGain(param.as_double_array()[0], param.as_double_array()[1]);
                            result.successful = true;
                            result_clik_gains = true;
                            RCLCPP_INFO(this->get_logger(), "Parameter \"%s\" updated.", param.get_name().c_str());
                        }
                        else
                        {
                            RCLCPP_WARN(this->get_logger(),
                            "The parameter \"%s\" shall have both elements greater than 0.0 and smaller than 2.0/dt (in this case: %lf).",
                            param.get_name().c_str(), (2.0/dt_));
                            result.successful = false;
                            result.reason = "CLIK gains shall be greater than 0.0 and smaller than 2.0/dt.";
                            result_clik_gains = false;
                        }
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "Set time step dt first.");
                        result_clik_gains = false;
                    }
                }
            }
        }
    }
    
    if (result_DOF == true and result_dt == true and result_clik_gains == true)
    {
        result.successful = true;
        result.reason = "All parameters have proper values.";
        // If all the parameters have been set, then PARAMETERS_OK_ becomes true.
        // If at some point, there will be an attempt to change some parameter to a wrong value,
        // that change won't be accepted. Therefore, the old set of proper parameters will continue
        // to be used. Thus there is no need to change PARAMETERS_OK_ to false in such a case.
        // (At least that's what I wanted to achieve)
        PARAMETERS_OK_ = true;
    }
    /*else
    {
        result.successful = false;
        result.reason = "Some of the parameters have wrong types or values.";
    }*/
    
    return result;
}

void InverseKinematicsNode::JointStateCallback(const sensor_msgs::msg::JointState & msg)
{
    // get the joint state, save it to class member variable
    
    if (msg.position.size() < DOF_)
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(),
            *this->get_clock(),
            5000,
            "To solve the inverse kinematics problem, there are %ld joint variables required, not %ld.",
            DOF_,
            msg.position.size());
    }
    else
    {
        // get the joint position vector "q" from the JointState type msg
        q_msr_ = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(msg.position.data(), DOF_);
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

void InverseKinematicsNode::JacobianStampedCallback(const rrlib_interfaces::msg::JacobianStamped & msg)
{
    // get the Jacobian, save it to class member variable
    if (msg.jacobian.jacobian_data.size() < (6 * DOF_))
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(),
            *this->get_clock(),
            5000,
            "To solve the inverse kinematics problem, the Jacobian shall be of the size 6x%ld.",
            DOF_);
    }
    else
    {
        // get the Jacobian matrix "jacobian_" from the JacobianStamped type msg
        //~ jacobian_ = Eigen::Map<const Eigen::Matrix<double, 6, Eigen::Dynamic>, Eigen::Unaligned>(msg.jacobian.jacobian_data.data());
        jacobian_ = Eigen::Map<const Eigen::MatrixXd>(&msg.jacobian.jacobian_data[0], 6, DOF_); //TODO: check this
        //~ jacobian_ = Eigen::Map<const Eigen::MatrixXd, Eigen::Unaligned>(msg.jacobian.jacobian_data.data(), 6, DOF_);
    }
}

void InverseKinematicsNode::CartesianTrajectoryCallback(const rrlib_interfaces::msg::CartesianTrajectoryPoint & msg)
{
    // get the desired end effector trajectory
    Eigen::VectorXd pose_des = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(msg.positions.data(), 7);
    Eigen::VectorXd vel_des = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(msg.velocities.data(), 6);
    
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
