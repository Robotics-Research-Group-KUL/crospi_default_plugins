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

    std::vector<float> init_joints;
    for (auto n : jsonchecker->asArray(config, "initial_joints")) {
        init_joints.push_back(jsonchecker->asDouble(n, ""));
    }

    initial_joints = init_joints;

    assert(initial_joints.size() == joint_pos.data.size());

    // joint_pos.data = initial_joints;
    std::copy(initial_joints.begin(), initial_joints.end(), joint_pos.data.begin());

    setpoint_joint_vel.data.fill( 0.0); //Initialize setpoint joint velocities to zero

    feedback_ptr = fb; //defined in RobotDriver super class.
    setpoint_ptr = sp; //defined in RobotDriver super class.
    name = robot_name; //defined in RobotDriver super class.
    std::cout << "Constructed object of simple_kinematic_simulation class with name: " << name << std::endl;

    // struct TripleBuffers {
    //     std::shared_ptr<triple_buffer_joint_type> joint_vel;
    //     std::shared_ptr<triple_buffer_joint_type> joint_torque;
    //     std::shared_ptr<triple_buffer_joint_type> joint_current;
    //     std::shared_ptr<triple_buffer_vector3_type> cartesian_pos;
    //     std::shared_ptr<triple_buffer_quaternion_type> cartesian_quat;
    //     std::shared_ptr<triple_buffer_screw_type> cartesian_twist;
    //     std::shared_ptr<triple_buffer_screw_type> cartesian_wrench;
    //     std::shared_ptr<triple_buffer_vector3_type> base_pos;
    //     std::shared_ptr<triple_buffer_quaternion_type> base_quat;
    //     std::shared_ptr<triple_buffer_screw_type> base_twist;
    // } triple_buffers_ptrs;  


    feedback_triple_buffers_ptrs.joint_pos = std::make_shared<triple_buffer_joint_type>(joint_pos);
    setpoint_triple_buffers_ptrs.joint_vel = std::make_shared<triple_buffer_joint_type>(setpoint_joint_vel);

    // using T = std::vector<double>;
    // boost::lockfree::spsc_value<T, boost::lockfree::allow_multiple_reads<false>> ch;
    // boost::lockfree::spsc_value< robotdrivers::FixedJointDataField<NUM_JOINTS> > triple_buffer_joint_type;

}

bool simple_kinematic_simulation::initialize()
{
    // joint_pos.data = initial_joints;
    std::copy(initial_joints.begin(), initial_joints.end(), joint_pos.data.begin());

    feedback_triple_buffers_ptrs.joint_pos->write(joint_pos);

    // feedback_ptr->mtx.lock();

    // feedback_ptr->joint.pos.data = joint_pos;
    // feedback_ptr->joint.pos.is_available = true;
    
    // //Uncomment to simulate the sensor data
    // // feedback_ptr->joint.vel.is_available = true;
    // // feedback_ptr->joint.torque.is_available = true;
    // // feedback_ptr->joint.current.is_available = true;

    // // feedback_ptr->cartesian.pos.is_available = true;
    // // feedback_ptr->cartesian.quat.is_available = true;
    // // feedback_ptr->cartesian.twist.is_available = true;
    // // feedback_ptr->cartesian.wrench.is_available = true;

    // // feedback_ptr->base.pos.is_available = true;
    // // feedback_ptr->base.quat.is_available = true;
    // // feedback_ptr->base.twist.is_available = true;

    // feedback_ptr->mtx.unlock();

    return true;
}


void simple_kinematic_simulation::update(volatile std::atomic<bool>& stopFlag)
{
    // feedback_ptr->mtx.lock();

    
    // setpoint_triple_buffers_ptrs.joint_vel->read(setpoint_joint_vel);
    readSetpointJointVelocity(setpoint_joint_vel);

    assert(joint_pos.data.size() == setpoint_joint_vel.data.size());

    for (unsigned int i=0; i<setpoint_joint_vel.data.size(); ++i) {
        joint_pos.data[i] += setpoint_joint_vel.data[i]*periodicity; //simple integration
    }

    feedback_triple_buffers_ptrs.joint_pos->write(joint_pos);
    // writeFeedbackJointPosition(joint_pos);

    // //Uncomment to simulate the sensor data
    // // feedback_ptr->joint.vel.data[0] = 10.1;
    // // feedback_ptr->joint.vel.data[1] = 10.2;
    // // feedback_ptr->joint.vel.data[2] = 10.3;
    // // feedback_ptr->joint.vel.data[3] = 10.4;

    // // feedback_ptr->joint.torque.data[0] = 20.1;
    // // feedback_ptr->joint.torque.data[1] = 20.2;
    // // feedback_ptr->joint.torque.data[2] = 20.3;
    // // feedback_ptr->joint.torque.data[3] = 20.4;

    // // feedback_ptr->joint.current.data[0] = 30.1;
    // // feedback_ptr->joint.current.data[1] = 30.2;
    // // feedback_ptr->joint.current.data[2] = 30.3;
    // // feedback_ptr->joint.current.data[3] = 30.4;

    // // feedback_ptr->cartesian.pos.x = 1.1;
    // // feedback_ptr->cartesian.pos.y = 1.2;
    // // feedback_ptr->cartesian.pos.z = 1.3;

    // // feedback_ptr->cartesian.quat.qx = 2.1;
    // // feedback_ptr->cartesian.quat.qy = 2.2;
    // // feedback_ptr->cartesian.quat.qz = 2.3;
    // // feedback_ptr->cartesian.quat.qw = 2.3;

    // // feedback_ptr->cartesian.twist.linear.x = 3.1;
    // // feedback_ptr->cartesian.twist.linear.y = 3.2;
    // // feedback_ptr->cartesian.twist.linear.z = 3.3;
    // // feedback_ptr->cartesian.twist.angular.x = 3.4;
    // // feedback_ptr->cartesian.twist.angular.y = 3.5;
    // // feedback_ptr->cartesian.twist.angular.z = 3.6;

    // // feedback_ptr->cartesian.wrench.linear.x = 0;
    // // feedback_ptr->cartesian.wrench.linear.y = 0;
    // // feedback_ptr->cartesian.wrench.linear.z = 0;
    // // feedback_ptr->cartesian.wrench.angular.x = 0;
    // // feedback_ptr->cartesian.wrench.angular.y = 0;
    // // feedback_ptr->cartesian.wrench.angular.z = 0;


    // // feedback_ptr->base.pos.x = 1.1;
    // // feedback_ptr->base.pos.y = 1.2;
    // // feedback_ptr->base.pos.z = 1.3;

    // // feedback_ptr->base.quat.qx = 2.1;
    // // feedback_ptr->base.quat.qy = 2.2;
    // // feedback_ptr->base.quat.qz = 2.3;
    // // feedback_ptr->base.quat.qw = 2.3;

    // // feedback_ptr->base.twist.linear.x = 3.1;
    // // feedback_ptr->base.twist.linear.y = 3.2;
    // // feedback_ptr->base.twist.linear.z = 3.3;
    // // feedback_ptr->base.twist.angular.x = 3.4;
    // // feedback_ptr->base.twist.angular.y = 3.5;
    // // feedback_ptr->base.twist.angular.z = 3.6;
    

    // setpoint_ptr->velocity.fs = etasl::OldData;
    // // std::cout << "vel val:" << setpoint_ptr->velocity.data[0] << " , " << setpoint_ptr->velocity.data[1] << " , "<< setpoint_ptr->velocity.data[2] << std::endl;
    // // std::cout << "Driver update has set all pos values to " << feedback_ptr->joint.pos.data[0] << std::endl;
    // // std::cout << "Driver update has set all pos values to " << this->periodicity << std::endl;
    // // std::cout << "Driver update has set all pos values to " << getName() << std::endl;


    // feedback_ptr->mtx.unlock();


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

// -----------Functions for thread communicating with this driver -------------- 

void simple_kinematic_simulation::writeSetpointJointVelocity(const std::vector<float>& sp ) noexcept{

    // std::cout << "1 im writing joint velocity setpoints" << std::endl;

    static robotdrivers::FixedJointDataField<NUM_JOINTS> buff{};

    assert(sp.size() == buff.data.size());
    
    for (unsigned int i=0; i<buff.data.size(); ++i) {
        buff.data[i] = sp[i]; //copy
    }
    

    #ifdef INCLUDE_TIMESTAMP_IN_DATA_STRUCTURE
    buff.timestamp = std::chrono::steady_clock::now();
    #endif

    setpoint_triple_buffers_ptrs.joint_vel->write(buff);

}

bool simple_kinematic_simulation::readFeedbackJointPosition(std::vector<float>& fb ) noexcept{
    // std::cout << "2 im reading joint position feedback" << std::endl;


    static robotdrivers::FixedJointDataField<NUM_JOINTS> joint_local_buff;
    bool ret = feedback_triple_buffers_ptrs.joint_pos->read(joint_local_buff);

    assert(fb.size() == joint_local_buff.data.size());

    for (unsigned int i=0; i<joint_local_buff.data.size(); ++i) {
        fb[i] = joint_local_buff.data[i]; //copy
    }

    return ret;

    // #ifdef INCLUDE_TIMESTAMP_IN_DATA_STRUCTURE
    // buff.timestamp = std::chrono::steady_clock::now();
    //TODO: benchmark
    // #endif
} 

// -----------Functions for this driver -------------- 

bool simple_kinematic_simulation::readSetpointJointVelocity(robotdrivers::FixedJointDataField<NUM_JOINTS>& sp ) noexcept{

    // std::cout << "3 im reading joint velocity setpoints" << std::endl;
    return setpoint_triple_buffers_ptrs.joint_vel->read(sp);

    // #ifdef INCLUDE_TIMESTAMP_IN_DATA_STRUCTURE
    // buff.timestamp = std::chrono::steady_clock::now();
    //TODO: benchmark
    // #endif
} 

void simple_kinematic_simulation::writeFeedbackJointPosition(const robotdrivers::FixedJointDataField<NUM_JOINTS>& fb ) noexcept{

    // std::cout << "4 im writing joint position feedback" << std::endl;
    feedback_triple_buffers_ptrs.joint_pos->write(fb);
}



simple_kinematic_simulation::~simple_kinematic_simulation() {
    //delete buffers
    // feedback_triple_buffers_ptrs.joint_pos.reset();
    // setpoint_triple_buffers_ptrs.joint_vel.reset();
    std::cout << "destructor called for simple_kinematic_simulation with name: " << name << std::endl;

};


// void* simple_kinematic_simulation::getSetpointJointVelocityBufferPtr(){return (void*)setpoint_triple_buffers_ptrs.joint_vel.get();};
// void* simple_kinematic_simulation::getJointPositionBufferPtr(){return (void*)feedback_triple_buffers_ptrs.joint_pos.get();};



} // namespace etasl


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(etasl::simple_kinematic_simulation, etasl::RobotDriver)
