#include "robotdrivers/ros_topics_driver_crospi.hpp"
#include <fmt/format.h>
#include <iostream>
#include <algorithm>

// include library for sleep
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace etasl {


ros_topics_driver_crospi::ros_topics_driver_crospi()
: received_feedback_(false)
{
    // DOF is not yet known at construction time; resizing happens in construct()
}

void ros_topics_driver_crospi::construct(std::string robot_name, 
                        const Json::Value& config,
                        std::shared_ptr<etasl::JsonChecker> jsonchecker)
{

    feedback_topic = jsonchecker->asString(config, "feedback_topic");
    command_topic = jsonchecker->asString(config, "command_topic");
    periodicity = jsonchecker->asDouble(config, "periodicity");
    
    // Read joint names from config and derive DOF from them
    robot_joints_.clear();
    for (auto n : jsonchecker->asArray(config, "robot_joints")) {
        robot_joints_.push_back(n.asString());
    }
    DOF = robot_joints_.size();
    name = robot_name; //defined in RobotDriver super class.

    AvailableFeedback available_fb{};
    available_fb.joint_pos = true;
    // Uncomment to enable additional feedback:
    // available_fb.joint_vel = true;
    // available_fb.joint_torque = true;

    constructPorts(DOF, available_fb); //Constructs all shared pointers and initialize data structures. Call after assigning available_feedback booleans.
    
    joint_pos_.data.resize(DOF, 0.0); //Initialize joint positions to zero
    setpoint_joint_vel_.data.resize(DOF, 0.0); //Initialize setpoint joint velocities to zero

    // Pre-allocate the command message
    command_msg.name.resize(DOF);
    command_msg.position.resize(DOF, 0.0);
    command_msg.velocity.resize(DOF, 0.0);

    std::cout << "Constructed robot driver with name: " << name 
              << ", DOF: " << DOF << std::endl;

}

bool ros_topics_driver_crospi::initialize()
{
    ros_node_ = std::make_shared<rclcpp::Node>("ros_topics_driver_crospi");

    auto qos = rclcpp::SensorDataQoS()
    .keep_last(1)
    .lifespan(rclcpp::Duration(0, 0))
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .durability(rclcpp::DurabilityPolicy::Volatile);
    
    feedback_subscriber_ = ros_node_->create_subscription<sensor_msgs::msg::JointState>(
        feedback_topic, qos,
        std::bind(&ros_topics_driver_crospi::feedbackCallback, this, std::placeholders::_1));

    command_publisher_ = ros_node_->create_publisher<sensor_msgs::msg::JointState>(
        command_topic, qos);

    executor_.add_node(ros_node_);

    // Set joint names in the command message
    command_msg.name = robot_joints_;

    // writeFeedbackJointPosition(joint_pos_);

    // Initialize command positions to the current feedback positions so we start from the actual robot state
    for (size_t i = 0; i < DOF; ++i) {
        command_msg.position[i] = joint_pos_.data[i];
    }

    return true;
}

void ros_topics_driver_crospi::update(volatile std::atomic<bool>& stopFlag)
{
    if(!received_feedback_) {
        // If feedback has not been received yet, we cannot update the command
        RCLCPP_WARN(ros_node_->get_logger(), "Feedback not received yet. Skipping update of control loop in Crospi.");
        executor_.spin_some(); //Spins the executor to process any incoming messages, including feedback
        return;
    }

    readSetpointJointVelocity(setpoint_joint_vel_);

    assert(joint_pos_.data.size() == setpoint_joint_vel_.data.size());

    // Publish joint velocity commands
    for (size_t i = 0; i < DOF; ++i) {
        command_msg.velocity[i] = setpoint_joint_vel_.data[i];
        command_msg.position[i] += setpoint_joint_vel_.data[i]*periodicity;
    }
    command_publisher_->publish(command_msg);

    executor_.spin_some();
    writeFeedbackJointPosition(joint_pos_);
}

void ros_topics_driver_crospi::feedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Match joints by name and extract positions
    for (size_t i = 0; i < robot_joints_.size(); ++i) {
        auto it = std::find(msg->name.begin(), msg->name.end(), robot_joints_[i]);
        if (it != msg->name.end()) {
            size_t index = std::distance(msg->name.begin(), it);
            if (index < msg->position.size()) {
                joint_pos_.data[i] = static_cast<float>(msg->position[index]);
            }
        }
    }
    if(!received_feedback_){
        // Initialize command positions to the current feedback positions so we start from the actual robot state
        for (size_t i = 0; i < DOF; ++i) {
            command_msg.position[i] = joint_pos_.data[i];
        }
        received_feedback_ = true;
    }
}

void ros_topics_driver_crospi::on_configure() {
    // std::cout << "entering on configure =======================" << std::endl;

}

void ros_topics_driver_crospi::on_activate() 
{

}

void ros_topics_driver_crospi::on_deactivate() {
    // std::cout << "entering on deactivate =======================" << std::endl;

}

void ros_topics_driver_crospi::on_cleanup() {
    // std::cout << "entering on cleanup =======================" << std::endl;

}


void ros_topics_driver_crospi::finalize() {
    std::cout << "finalize() called =======================" << std::endl;

    // Send zero velocity command to stop the robot arm safely
    for (size_t i = 0; i < DOF; ++i) {
        command_msg.velocity[i] = 0.0;
    }
    command_publisher_->publish(command_msg);
    // Example that uses wait_for_all_acked: https://github.com/ros2/examples/pull/316/files
    command_publisher_->wait_for_all_acked(std::chrono::milliseconds(500)); //Blocks until all subscribers have gotten the message with a timeout
}



ros_topics_driver_crospi::~ros_topics_driver_crospi() {

};



} // namespace etasl


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(etasl::ros_topics_driver_crospi, etasl::RobotDriver)
