#pragma once

#include <string>


#include "robot_interfacing_utils/robotsimulator.hpp"
// #include "robot_interfacing_utils/controlmodes_enum.hpp"
// #include "robot_interfacing_utils/robot_data_structures.hpp"



namespace etasl {

class simple_kinematic_simulation : public RobotSimulator {
    public:
        typedef std::shared_ptr<simple_kinematic_simulation> SharedPtr;


    private:
    //    static constexpr int NUM_JOINTS = 6; //TODO: for now assume 6 joints
        
        // FeedbackMsg* feedback_ptr; Defined in super class RobotSimulator at header file robotsimulator.hpp
        // SetpointMsg* setpoint_ptr; Defined in super class RobotSimulator at header file robotsimulator.hpp
        // std::string name;; Defined in super class RobotSimulator at header file robotsimulator.hpp

        double periodicity;

        std::vector<float> initial_joints;
        robotdrivers::DynamicJointDataField setpoint_joint_vel;
        robotdrivers::DynamicJointDataField joint_pos;
        // robotdrivers::FixedJointDataField<NUM_JOINTS> setpoint_joint_vel;
        // robotdrivers::FixedJointDataField<NUM_JOINTS> joint_pos;



        // bool readSetpointJointVelocity(robotdrivers::FixedJointDataField<NUM_JOINTS>& sp ) noexcept;
        // void writeFeedbackJointPosition(const robotdrivers::FixedJointDataField<NUM_JOINTS>& fb ) noexcept;
        // bool readSetpointJointVelocity(robotdrivers::DynamicJointDataField& sp ) noexcept;
        // void writeFeedbackJointPosition(const robotdrivers::DynamicJointDataField& fb ) noexcept;
        

    public:
        simple_kinematic_simulation();

        virtual void construct(std::string robot_name, 
                        robotdrivers::FeedbackMsg* fb, 
                        robotdrivers::SetpointMsg* sp,
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

        // void writeSetpointJointVelocity(const std::vector<float>& sp ) noexcept override;
        // bool readFeedbackJointPosition(std::vector<float>& fb ) noexcept override;


        // virtual const std::string& getName() const override;

        virtual ~simple_kinematic_simulation();





};

} // namespace etasl
