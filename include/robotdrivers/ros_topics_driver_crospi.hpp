#pragma once

#include <string>


#include "robot_interfacing_utils/robotdriver.hpp"
#include "robot_interfacing_utils/controlmodes_enum.hpp"

// include ros and joint state messages
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace etasl {

class ros_topics_driver_crospi : public RobotDriver {
    public:
        typedef std::shared_ptr<ros_topics_driver_crospi> SharedPtr;


    private:
        
        // FeedbackMsg* feedback_ptr; Defined in super class RobotDriver at header file robotdriver.hpp
        // SetpointMsg* setpoint_ptr; Defined in super class RobotDriver at header file robotdriver.hpp
        // std::string name;; Defined in super class RobotDriver at header file robotdriver.hpp

        bool received_feedback_; // Flag to indicate if feedback has been received at least once
        double periodicity;
        std::string feedback_topic;
        std::string command_topic;
        size_t DOF; // Configurable number of joints, read from JSON config
        std::vector<std::string> robot_joints_; // Names of the joints, read from JSON config

        robotdrivers::DynamicJointDataField setpoint_joint_vel_;
        robotdrivers::DynamicJointDataField joint_pos_;

        std::shared_ptr<rclcpp::Node> ros_node_;
        rclcpp::executors::SingleThreadedExecutor executor_;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_publisher_;
        sensor_msgs::msg::JointState command_msg;
        

    public:
        ros_topics_driver_crospi();

        virtual void construct(std::string robot_name, 
                        const Json::Value& config,
                        std::shared_ptr<etasl::JsonChecker> jsonchecker) override;

        /**
         * will only return true if it has received values for all the joints named in jnames.
        */
        virtual bool initialize() override;

        virtual void update(volatile std::atomic<bool>& stopFlag) override;

        virtual void on_configure() override;

        virtual void on_activate() override;

        virtual void on_deactivate() override;

        virtual void on_cleanup() override;

        virtual void finalize() override;
        
        void feedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
        // virtual const std::string& getName() const override;

        virtual ~ros_topics_driver_crospi();
};

} // namespace etasl
