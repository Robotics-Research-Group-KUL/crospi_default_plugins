//  Copyright (c) 2025 KU Leuven, Belgium
//
//  Author: Santiago Iregui
//  email: <santiago.iregui@kuleuven.be>
//
//  GNU Lesser General Public License Usage
//  Alternatively, this file may be used under the terms of the GNU Lesser
//  General Public License version 3 as published by the Free Software
//  Foundation and appearing in the file LICENSE.LGPLv3 included in the
//  packaging of this file. Please review the following information to
//  ensure the GNU Lesser General Public License version 3 requirements
//  will be met: https://www.gnu.org/licenses/lgpl.html.
// 
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.

#include "inputhandlers/jointstateinputhandler.hpp"
// #include "rclcpp/wait_for_message.hpp"
#include <fmt/format.h>

namespace etasl {


JointStateInputHandler::JointStateInputHandler()
    : counter(0)
    , initialized(false)
    , activated(false)
{
    //Constructor with no arguments needed for plugin registration
}


bool JointStateInputHandler::construct(
    std::string _name,
    rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
    const Json::Value& parameters,
    std::shared_ptr<etasl::JsonChecker> jsonchecker)
{
    node = _node;
    topicname = jsonchecker->asString(parameters, "topic-name");

    name = fmt::format("{}({})",_name, topicname);
        
    depth = jsonchecker->asInt(parameters, "depth");
    nroftries = jsonchecker->asInt(parameters, "number_of_tries");
    number_of_joints = jsonchecker->asInt(parameters, "number_of_joints");

    when_unpublished = jsonchecker->asString(parameters, "when_unpublished");

    varname_prefix = jsonchecker->asString(parameters, "varname_prefix");




    Json::Value default_msg_json = jsonchecker->asArray(parameters, "default_joint_states");

    default_msg.resize(default_msg_json.size(), 0.0);
    for (int i = 0; i < default_msg_json.size(); ++i) {
        default_msg[i] = default_msg_json[i].asDouble();
    }

    if(default_msg.size() != number_of_joints){
        RCLCPP_ERROR(node->get_logger(), fmt::format("The provided default message for input handler {} has a different number of values than expected. Consider providing a default message with {} values or changing the when_unpublished policy to throw_error.", name, number_of_joints).c_str());
        return false;
    }


    // cbg = node->create_callback_group()
    // sub = node->create_subscription<MsgType>(_topic_name, rclcpp::QoS(10), cb);

    input_msg.fs = NoData;

    std::cout << "JointStateInputHandler constructed with topic name: " << topicname << std::endl;

    return true;
    
}

bool JointStateInputHandler::initialize(
    Context::Ptr ctx,
    const std::vector<std::string>& jnames,
    const std::vector<std::string>& fnames,
    Eigen::VectorXd& jpos,
    Eigen::VectorXd& fpos)
{



    if(input_msg.fs == NoData && when_unpublished=="use_default"){
        input_msg.fs = NewData; //Fakes new Data         
    }
    // else if(input_msg.fs == NoData && when_unpublished=="throw_error"){
    //     input_msg.data = default_msg;
    // }


    input_jpos_val.resize(number_of_joints);
    inps.resize(number_of_joints);

    input_msg.data.position.resize(number_of_joints, 0.0);
    input_msg.data.velocity.resize(number_of_joints, 0.0);
    input_msg.data.effort.resize(number_of_joints, 0.0);
    
    for (int i = 0; i < default_msg.size(); ++i) {
        input_msg.data.position[i] = default_msg[i];
        input_jpos_val[i] = default_msg[i];
    }

    using namespace std::chrono_literals;
    using namespace std::placeholders;
    auto cb = std::bind(&JointStateInputHandler::on_new_message, this, _1);
    auto qos = rclcpp::SensorDataQoS().keep_last(depth).lifespan(100ms);
    sub = node->create_subscription<MsgType>(topicname, qos, cb);

    if (!sub) {
        RCLCPP_FATAL(node->get_logger(), "Could not create subscriber for input handler '%s'.", name.c_str());
        return false;
    }



    initialized = true;
    RCLCPP_INFO(node->get_logger(), "jointstateinputhandler has been initialized correctly ________________________");
    return true;
}

void JointStateInputHandler::on_new_message(const JointStateInputHandler::MsgType& msg)
{
    input_msg.fs = NewData;
    input_msg.data = msg;
}

void JointStateInputHandler::update(
    double time,
    const std::vector<std::string>& jnames,
    Eigen::VectorXd& jpos,
    const std::vector<std::string>& fnames,
    Eigen::VectorXd& fpos)
{
    // if(activated){
    //     std::cout << "it is activated ////////////////////////////////" << std::endl;
    //     std::cout << "fs: " << input_msg.fs << std::endl;
    //     std::cout << "when_unpublished: " << when_unpublished  << std::endl;
    // }
    if(activated && input_msg.fs == NoData && when_unpublished=="throw_error"){

        if (counter > nroftries){
            RCLCPP_ERROR(node->get_logger(), fmt::format("jointstateinputhandler could not initialize since no data has been published in topic {} and therefore the node is shutting down. Consider changing when_unpublished policy of inputhandler to use_default if desired.",name).c_str());
            rclcpp::shutdown();
        }
        else{
            counter++;
            // std::cout << "Counter +++++: " << counter<< std::endl;
        }
        return;
    }
    else if(activated && input_msg.fs == NewData){
        consume_data(true); 
    }
    // RCLCPP_INFO(node->get_logger(), "JointStateInputHander::update()");
}

void JointStateInputHandler::on_activate(Context::Ptr ctx,
    const std::vector<std::string>& jnames,
    const std::vector<std::string>& fnames,
    boost::shared_ptr<solver> slv) 
{
    // std::cout << "entering on activate =======================" << std::endl;
    if(!initialized){
        RCLCPP_WARN(node->get_logger(), "The JointStateInputHandler cannot be activated since it has not been initialized yet");
        return;
    }
    if(activated){
        RCLCPP_WARN(node->get_logger(), "The JointStateInputHandler cannot be activated since it has already been activated. Call on_deactivate first.");
        return;
    }

    
    
    bool all_variables_exist = true;
    
    inps.resize(number_of_joints);
    for (size_t i = 0; i < number_of_joints; ++i) {
        inps[i] = ctx->getInputChannel<double>(fmt::format("{}_{}",varname_prefix, i+1).c_str());
        if (!inps[i]) {
            RCLCPP_INFO(node->get_logger(), fmt::format("No input channel with the name {} was created in the LUA specification, and thus topic {} will be ignored. If desired, this can be created with with createInputChannelTwist function.",varname_prefix, name).c_str());
            activated = false;
        } 
    }

    if (all_variables_exist) {
        // inps->setJacobian(time_ndx, default_value_kdl );
        consume_data(false); 
        activated = true;
    }
}

void JointStateInputHandler::on_deactivate(Context::Ptr ctx) {
    // std::cout << "entering on deactivate =======================" << std::endl;

    for (size_t i = 0; i < number_of_joints; ++i) {
        inps[i] = nullptr;
    }
    activated=false;
}

const std::string& JointStateInputHandler::getName() const
{
    return name;
}

void JointStateInputHandler::consume_data(const bool& make_old_data)
{
    for (size_t i = 0; i < inps.size(); ++i) {

        if (inps[i]) { //Checks if variable exists, i.e. there is such a variable used in the etasl task specification
            input_jpos_val[i] = input_msg.data.position[i];
    
            inps[i]->setValue(input_jpos_val[i]);
            if (make_old_data){
                input_msg.fs = OldData;
            }
        }
    }
}

JointStateInputHandler::~JointStateInputHandler() {

};



} // namespace etasl


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(etasl::JointStateInputHandler, etasl::InputHandler)
