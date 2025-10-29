#include "simulators/simple_kinematic_simulation.hpp"
#include <fmt/format.h>
#include <iostream>

namespace etasl {


simple_kinematic_simulation::simple_kinematic_simulation()
    // : periodicity(periodicity_val)
    // , initial_joints(init_joints)
    // , joint_pos(init_joints)
{
}

void simple_kinematic_simulation::construct(std::string robot_name, 
                        const Json::Value& config,
                        std::shared_ptr<etasl::JsonChecker> jsonchecker)
{

    periodicity = jsonchecker->asDouble(config, "periodicity");

    std::vector<float> init_joints;
    for (auto n : jsonchecker->asArray(config, "initial_joints")) {
        init_joints.push_back(jsonchecker->asDouble(n, ""));
    }

    initial_joints = init_joints;
    joint_pos.data = init_joints;

    assert(initial_joints.size() == joint_pos.data.size());

    // joint_pos.data = initial_joints;
    // std::copy(initial_joints.begin(), initial_joints.end(), joint_pos.data.begin());

    // setpoint_joint_vel.data.fill( 0.0); //Initialize setpoint joint velocities to zero
    setpoint_joint_vel.data.resize(initial_joints.size(), 0.0); //Initialize setpoint joint velocities to zero

    name = robot_name; //defined in RobotDriver super class.
    std::cout << "Constructed object of simple_kinematic_simulation class with name: " << name << std::endl;

    AvailableFeedback available_fb{};
    available_fb.joint_pos = true;

    if(jsonchecker->is_member(config, "name_expr_joint_vel")){available_fb.joint_vel= true;}
    if(jsonchecker->is_member(config, "name_expr_joint_torque")){available_fb.joint_torque= true;}
    if(jsonchecker->is_member(config, "name_expr_joint_current")){available_fb.joint_current= true;}
    if(jsonchecker->is_member(config, "name_expr_cartesian_pos")){available_fb.cartesian_pos= true;}
    if(jsonchecker->is_member(config, "name_expr_cartesian_quat")){available_fb.cartesian_quat= true;}
    if(jsonchecker->is_member(config, "name_expr_cartesian_twist")){available_fb.cartesian_twist= true;}
    if(jsonchecker->is_member(config, "name_expr_cartesian_wrench")){available_fb.cartesian_wrench= true;}
    if(jsonchecker->is_member(config, "name_expr_base_pos")){available_fb.base_pos= true;}
    if(jsonchecker->is_member(config, "name_expr_base_quat")){available_fb.base_quat= true;}
    if(jsonchecker->is_member(config, "name_expr_base_twist")){available_fb.base_twist= true;}

    // available_fb.joint_torque= true;
    // available_fb.joint_current= true;
    // available_fb.cartesian_pos= true;
    // available_fb.cartesian_quat= true;
    // available_fb.cartesian_twist= true;
    // available_fb.cartesian_wrench= true;
    // available_fb.base_pos= true;
    // available_fb.base_quat= true;
    // available_fb.base_twist= true;

    constructPorts(initial_joints.size(), available_fb); //Constructs all shared pointers and initialize data structures. Call after assigning available_feedback booleans.

}

bool simple_kinematic_simulation::initialize()
{
    // joint_pos.data = initial_joints;
    std::copy(initial_joints.begin(), initial_joints.end(), joint_pos.data.begin());

    writeFeedbackJointPosition(joint_pos);
    // writeFeedbackJointVelocity(robotdrivers::DynamicJointDataField(initial_joints.size()));
    

    return true;
}


void simple_kinematic_simulation::update(volatile std::atomic<bool>& stopFlag)
{
    // feedback_ptr->mtx.lock();

    
    readSetpointJointVelocity(setpoint_joint_vel);

    assert(joint_pos.data.size() == setpoint_joint_vel.data.size());

    for (unsigned int i=0; i<setpoint_joint_vel.data.size(); ++i) {
        joint_pos.data[i] += setpoint_joint_vel.data[i]*periodicity; //simple integration

        // std::cout << setpoint_joint_vel.data[i] << ", ";
    }
    // std::cout << std::endl;

    writeFeedbackJointPosition(joint_pos);



    if (available_feedback.joint_vel) {
        static robotdrivers::DynamicJointDataField joint_velocity(joint_pos.data.size());
        joint_velocity.data[0] += 0.01f;
        writeFeedbackJointVelocity(joint_velocity);
    }

    if (available_feedback.joint_torque) {
        static robotdrivers::DynamicJointDataField joint_torque(joint_pos.data.size());
        joint_torque.data[0] += 0.02f;
        writeFeedbackJointTorque(joint_torque);
    }

    if (available_feedback.joint_current) {
        static robotdrivers::DynamicJointDataField join_current(joint_pos.data.size());
        join_current.data[0] += 0.03f;
        writeFeedbackJointCurrent(join_current);
    }


    // //Write Cartesian data for simulation and testing purposes


    if (available_feedback.cartesian_pos) {
        static robotdrivers::Vector3Field cartesian_pos{};
        cartesian_pos.x += 0.02f;
        writeFeedbackCartesianPosition(cartesian_pos);
    }

    if (available_feedback.cartesian_quat) {
        static robotdrivers::QuaternionField cartesian_quat{};
        cartesian_quat.qx += 0.03f;
        writeFeedbackCartesianQuaternion(cartesian_quat);
    }

    if (available_feedback.cartesian_twist) {
        static robotdrivers::ScrewField cartesian_twist{};
        cartesian_twist.linear.x += 0.04f;
        writeFeedbackCartesianTwist(cartesian_twist);
    }

    if (available_feedback.cartesian_wrench) {
        static robotdrivers::ScrewField cartesian_wrench{};
        cartesian_wrench.linear.x += 0.01f;
        writeFeedbackCartesianWrench(cartesian_wrench);
    }

    if (available_feedback.base_pos) {
        static robotdrivers::Vector3Field base_pos{};
        base_pos.x += 0.05f;
        writeFeedbackBasePosition(base_pos);
    }
    
    if (available_feedback.base_quat) {
        static robotdrivers::QuaternionField base_quat{};
        base_quat.qx += 0.06f;
        writeFeedbackBaseQuaternion(base_quat);
    }

    if (available_feedback.base_twist) {
        static robotdrivers::ScrewField base_twist{};
        base_twist.linear.x += 0.07f;
        writeFeedbackBaseTwist(base_twist);
    }

}

void simple_kinematic_simulation::on_configure() {
    // std::cout << "entering on configure =======================" << std::endl;

}

void simple_kinematic_simulation::on_activate() 
{


}

void simple_kinematic_simulation::on_deactivate() {
    // std::cout << "entering on deactivate =======================" << std::endl;

}

void simple_kinematic_simulation::on_cleanup() {
    // std::cout << "entering on cleanup =======================" << std::endl;

}


void simple_kinematic_simulation::finalize() {
    std::cout << "finalize() called =======================" << std::endl;

}


simple_kinematic_simulation::~simple_kinematic_simulation() {

    std::cout << "destructor called for simple_kinematic_simulation with name: " << name << std::endl;

};




} // namespace etasl


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(etasl::simple_kinematic_simulation, etasl::RobotDriver)
