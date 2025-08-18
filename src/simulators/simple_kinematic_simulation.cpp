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
                        FeedbackMsg* fb, 
                        SetpointMsg* sp,
                        const Json::Value& config,
                        std::shared_ptr<etasl::JsonChecker> jsonchecker)
{

    periodicity = jsonchecker->asDouble(config, "periodicity");

    std::vector<double> init_joints;
    for (auto n : jsonchecker->asArray(config, "initial_joints")) {
        init_joints.push_back(jsonchecker->asDouble(n, ""));
    }

    initial_joints = init_joints;
    joint_pos = initial_joints;

    feedback_ptr = fb; //defined in RobotDriver super class.
    setpoint_ptr = sp; //defined in RobotDriver super class.
    name = robot_name; //defined in RobotDriver super class.
    std::cout << "Constructed object of simple_kinematic_simulation class with name: " << name << std::endl;

}

bool simple_kinematic_simulation::initialize()
{
    joint_pos = initial_joints;

    feedback_ptr->mtx.lock();
    // setpoint_ptr->mtx.lock();

    feedback_ptr->joint.pos.data = joint_pos;
    feedback_ptr->joint.pos.is_available = true;
    
    //Uncomment to simulate the sensor data
    // feedback_ptr->joint.vel.is_available = true;
    // feedback_ptr->joint.torque.is_available = true;
    // feedback_ptr->joint.current.is_available = true;

    // feedback_ptr->cartesian.pos.is_available = true;
    // feedback_ptr->cartesian.quat.is_available = true;
    // feedback_ptr->cartesian.twist.is_available = true;
    // feedback_ptr->cartesian.wrench.is_available = true;

    // feedback_ptr->base.pos.is_available = true;
    // feedback_ptr->base.quat.is_available = true;
    // feedback_ptr->base.twist.is_available = true;

    // setpoint_ptr->mtx.unlock();
    feedback_ptr->mtx.unlock();

    return true;
}


void simple_kinematic_simulation::update(volatile std::atomic<bool>& stopFlag)
{
    feedback_ptr->mtx.lock();
    // setpoint_ptr->mtx.lock();

    assert(feedback_ptr->joint.pos.data.size() == setpoint_ptr->velocity.data.size());

    for (unsigned int i=0; i<feedback_ptr->joint.pos.data.size(); ++i) {
        joint_pos[i] += setpoint_ptr->velocity.data[i]*periodicity; //simple integration
        // joint_pos[i] += setpoint_ptr->velocity.data[i]*0.0000005; //simple integration
        // joint_pos[i] += 0.00000001; //test
        feedback_ptr->joint.pos.data[i] = joint_pos[i];
    }

    //Uncomment to simulate the sensor data
    // feedback_ptr->joint.vel.data[0] = 10.1;
    // feedback_ptr->joint.vel.data[1] = 10.2;
    // feedback_ptr->joint.vel.data[2] = 10.3;
    // feedback_ptr->joint.vel.data[3] = 10.4;

    // feedback_ptr->joint.torque.data[0] = 20.1;
    // feedback_ptr->joint.torque.data[1] = 20.2;
    // feedback_ptr->joint.torque.data[2] = 20.3;
    // feedback_ptr->joint.torque.data[3] = 20.4;

    // feedback_ptr->joint.current.data[0] = 30.1;
    // feedback_ptr->joint.current.data[1] = 30.2;
    // feedback_ptr->joint.current.data[2] = 30.3;
    // feedback_ptr->joint.current.data[3] = 30.4;

    // feedback_ptr->cartesian.pos.x = 1.1;
    // feedback_ptr->cartesian.pos.y = 1.2;
    // feedback_ptr->cartesian.pos.z = 1.3;

    // feedback_ptr->cartesian.quat.qx = 2.1;
    // feedback_ptr->cartesian.quat.qy = 2.2;
    // feedback_ptr->cartesian.quat.qz = 2.3;
    // feedback_ptr->cartesian.quat.qw = 2.3;

    // feedback_ptr->cartesian.twist.linear.x = 3.1;
    // feedback_ptr->cartesian.twist.linear.y = 3.2;
    // feedback_ptr->cartesian.twist.linear.z = 3.3;
    // feedback_ptr->cartesian.twist.angular.x = 3.4;
    // feedback_ptr->cartesian.twist.angular.y = 3.5;
    // feedback_ptr->cartesian.twist.angular.z = 3.6;

    // feedback_ptr->cartesian.wrench.linear.x = 0;
    // feedback_ptr->cartesian.wrench.linear.y = 0;
    // feedback_ptr->cartesian.wrench.linear.z = 0;
    // feedback_ptr->cartesian.wrench.angular.x = 0;
    // feedback_ptr->cartesian.wrench.angular.y = 0;
    // feedback_ptr->cartesian.wrench.angular.z = 0;


    // feedback_ptr->base.pos.x = 1.1;
    // feedback_ptr->base.pos.y = 1.2;
    // feedback_ptr->base.pos.z = 1.3;

    // feedback_ptr->base.quat.qx = 2.1;
    // feedback_ptr->base.quat.qy = 2.2;
    // feedback_ptr->base.quat.qz = 2.3;
    // feedback_ptr->base.quat.qw = 2.3;

    // feedback_ptr->base.twist.linear.x = 3.1;
    // feedback_ptr->base.twist.linear.y = 3.2;
    // feedback_ptr->base.twist.linear.z = 3.3;
    // feedback_ptr->base.twist.angular.x = 3.4;
    // feedback_ptr->base.twist.angular.y = 3.5;
    // feedback_ptr->base.twist.angular.z = 3.6;
    

    setpoint_ptr->velocity.fs = etasl::OldData;
    // std::cout << "vel val:" << setpoint_ptr->velocity.data[0] << " , " << setpoint_ptr->velocity.data[1] << " , "<< setpoint_ptr->velocity.data[2] << std::endl;


    // std::cout << "Driver update has set all pos values to " << feedback_ptr->joint.pos.data[0] << std::endl;
    // std::cout << "Driver update has set all pos values to " << this->periodicity << std::endl;
    // std::cout << "Driver update has set all pos values to " << getName() << std::endl;

    // setpoint_ptr->mtx.unlock();
    feedback_ptr->mtx.unlock();


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

};



} // namespace etasl


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(etasl::simple_kinematic_simulation, etasl::RobotDriver)
