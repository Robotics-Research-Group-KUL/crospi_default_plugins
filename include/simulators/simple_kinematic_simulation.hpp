#pragma once

#include <string>


#include "robot_interfacing_utils/robotsimulator.hpp"
#include "robot_interfacing_utils/controlmodes_enum.hpp"
#include "robot_interfacing_utils/robot_data_structures.hpp"



namespace etasl {

class simple_kinematic_simulation : public RobotSimulator {
    public:
        typedef std::shared_ptr<simple_kinematic_simulation> SharedPtr;


    private:
       static constexpr int NUM_JOINTS = 6; //TODO: for now assume 6 joints
        
        // FeedbackMsg* feedback_ptr; Defined in super class RobotSimulator at header file robotsimulator.hpp
        // SetpointMsg* setpoint_ptr; Defined in super class RobotSimulator at header file robotsimulator.hpp
        // std::string name;; Defined in super class RobotSimulator at header file robotsimulator.hpp

        double periodicity;
        ControlMode::ControlMode control_mode;

        std::vector<float> initial_joints;
        robotdrivers::FixedJointDataField<NUM_JOINTS> setpoint_joint_vel;
        robotdrivers::FixedJointDataField<NUM_JOINTS> joint_pos;

        typedef boost::lockfree::spsc_value< robotdrivers::FixedJointDataField<NUM_JOINTS> > triple_buffer_joint_type;
        typedef boost::lockfree::spsc_value< robotdrivers::Vector3Field > triple_buffer_vector3_type;
        typedef boost::lockfree::spsc_value< robotdrivers::QuaternionField2 > triple_buffer_quaternion_type;
        typedef boost::lockfree::spsc_value< robotdrivers::ScrewField2 > triple_buffer_screw_type;


        
        struct FeedbackTripleBuffers {
            std::shared_ptr<triple_buffer_joint_type> joint_pos;
            std::shared_ptr<triple_buffer_joint_type> joint_vel;
            std::shared_ptr<triple_buffer_joint_type> joint_torque;
            std::shared_ptr<triple_buffer_joint_type> joint_current;
            std::shared_ptr<triple_buffer_vector3_type> cartesian_pos;
            std::shared_ptr<triple_buffer_quaternion_type> cartesian_quat;
            std::shared_ptr<triple_buffer_screw_type> cartesian_twist;
            std::shared_ptr<triple_buffer_screw_type> cartesian_wrench;
            std::shared_ptr<triple_buffer_vector3_type> base_pos;
            std::shared_ptr<triple_buffer_quaternion_type> base_quat;
            std::shared_ptr<triple_buffer_screw_type> base_twist;
        } feedback_triple_buffers_ptrs;  

        struct SetpointTripleBuffers {
            std::shared_ptr<triple_buffer_joint_type> joint_vel;
        } setpoint_triple_buffers_ptrs;  

        bool readSetpointJointVelocity(robotdrivers::FixedJointDataField<NUM_JOINTS>& sp ) noexcept;
        void writeFeedbackJointPosition(const robotdrivers::FixedJointDataField<NUM_JOINTS>& fb ) noexcept;
        

    public:
        simple_kinematic_simulation();

        virtual void construct(std::string robot_name, 
                        FeedbackMsg* fb, 
                        SetpointMsg* sp,
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

        // virtual void* getSetpointJointVelocityBufferPtr() override;
        // virtual void* getJointPositionBufferPtr() override;

        void writeSetpointJointVelocity(const std::vector<float>& sp ) noexcept override;
        bool readFeedbackJointPosition(std::vector<float>& fb ) noexcept override;


        // virtual const std::string& getName() const override;

        virtual ~simple_kinematic_simulation();





};

} // namespace etasl
