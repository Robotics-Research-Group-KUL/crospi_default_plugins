#pragma once

#include <string>


#include "robot_interfacing_utils/robotdriver.hpp"
#include "robot_interfacing_utils/controlmodes_enum.hpp"


namespace etasl {

class simple_kinematic_simulation : public RobotDriver {
    public:
        typedef std::shared_ptr<simple_kinematic_simulation> SharedPtr;


    private:
        
        // FeedbackMsg* feedback_ptr; Defined in super class RobotDriver at header file robotdriver.hpp
        // SetpointMsg* setpoint_ptr; Defined in super class RobotDriver at header file robotdriver.hpp
        // std::string name;; Defined in super class RobotDriver at header file robotdriver.hpp

        double periodicity;
        ControlMode::ControlMode control_mode;

        std::vector<double> initial_joints;
        std::vector<double> joint_pos;
        

    public:
        simple_kinematic_simulation();

        virtual void construct(std::string robot_name, 
                        FeedbackMsg* fb, 
                        SetpointMsg* sp,
                        const Json::Value& config,
                        boost::shared_ptr<etasl::JsonChecker> jsonchecker) override;

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

        // virtual const std::string& getName() const override;

        virtual ~simple_kinematic_simulation();
};

} // namespace etasl
