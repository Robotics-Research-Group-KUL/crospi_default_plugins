#include "inputhandlers/wrenchinputhandler.hpp"
// #include "rclcpp/wait_for_message.hpp"
#include <fmt/format.h>

namespace etasl {


WrenchInputHandler::WrenchInputHandler()
    : wrench(KDL::Wrench::Zero())
    , counter(0)
    , initialized(false)
    , activated(false)
{
}

bool WrenchInputHandler::construct(
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

    when_unpublished = jsonchecker->asString(parameters, "when_unpublished");

    varname = jsonchecker->asString(parameters, "varname");


    default_msg.force.x = jsonchecker->asDouble(parameters, "default_wrench/force/x");
    default_msg.force.y = jsonchecker->asDouble(parameters, "default_wrench/force/y");
    default_msg.force.z = jsonchecker->asDouble(parameters, "default_wrench/force/z");
    default_msg.torque.x = jsonchecker->asDouble(parameters, "default_wrench/torque/x");
    default_msg.torque.y = jsonchecker->asDouble(parameters, "default_wrench/torque/y");
    default_msg.torque.z = jsonchecker->asDouble(parameters, "default_wrench/torque/z");

    // cbg = node->create_callback_group()
    // sub = node->create_subscription<MsgType>(_topic_name, rclcpp::QoS(10), cb);

    input_msg.fs = NoData;
    
    // input_msg.data = default_msg;

    return true;
}

bool WrenchInputHandler::initialize(
    Context::Ptr ctx,
    const std::vector<std::string>& jnames,
    const std::vector<std::string>& fnames,
    Eigen::VectorXd& jpos,
    Eigen::VectorXd& fpos)
{


    if(input_msg.fs == NoData && when_unpublished=="use_default"){
        input_msg.fs = NewData; //Fakes new Data 
        input_msg.data = default_msg;
    }
    else if(input_msg.fs == NoData && when_unpublished=="throw_error"){
        input_msg.data = default_msg;
    }

    using namespace std::chrono_literals;
    using namespace std::placeholders;
    auto cb = std::bind(&WrenchInputHandler::on_new_message, this, _1);
    auto qos = rclcpp::SensorDataQoS().keep_last(depth).lifespan(100ms);
    sub = node->create_subscription<MsgType>(topicname, qos, cb);
    if (!sub) {
        //Log with name
        RCLCPP_FATAL(node->get_logger(), "Could not create subscriber for input handler '%s'.", name.c_str());
        return false;
    }

    initialized = true;
    RCLCPP_INFO(node->get_logger(), "wrenchinputhandler has been initialized correctly ________________________");
    return true;
}

void WrenchInputHandler::on_new_message(const WrenchInputHandler::MsgType& msg)
{
    input_msg.fs = NewData;
    input_msg.data = msg;
}

void WrenchInputHandler::update(
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
            RCLCPP_ERROR(node->get_logger(), fmt::format("wrenchinputhandler could not initialize since no data has been published in topic {} and therefore the node is shutting down. Consider changing when_unpublished policy of inputhandler to use_default if desired.",name).c_str());
            rclcpp::shutdown();
        }
        else{
            counter++;
            std::cout << "Counter +++++: " << counter<< std::endl;
        }
        return;
    }
    else if(activated && input_msg.fs == NewData){
        consume_data(true); 
    }
    // RCLCPP_INFO(node->get_logger(), "JointStateInputHander::update()");
}

void WrenchInputHandler::on_activate(Context::Ptr ctx,
    const std::vector<std::string>& jnames,
    const std::vector<std::string>& fnames) 
{
    if(!initialized){
        RCLCPP_WARN(node->get_logger(), "The WrenchInputHandler cannot be activated since it has not been initialized yet");
        return;
    }
    if(activated){
        RCLCPP_WARN(node->get_logger(), "The WrenchInputHandler cannot be activated since it has already been activated. Call on_deactivate first.");
        return;
    }

    inp = ctx->getInputChannel<KDL::Wrench>(varname); 
    if (!inp) {
        RCLCPP_INFO(node->get_logger(), fmt::format("No wrench input channel with the name {} was created in the LUA specification, and thus topic {} will be ignored. If desired, this can be created with with createInputChannelWrench function.",varname, name).c_str());
        activated = false;
    } 
    else {
        consume_data(false); 
        activated = true;
    }
}

void WrenchInputHandler::on_deactivate(Context::Ptr ctx) {
    // std::cout << "entering on deactivate =======================" << std::endl;

    inp = nullptr;
    activated=false;
}

const std::string& WrenchInputHandler::getName() const
{
    return name;
}

void WrenchInputHandler::consume_data(const bool& make_old_data)
{
    if (inp) {
        wrench.force[0] = input_msg.data.force.x;
        wrench.force[1] = input_msg.data.force.y;
        wrench.force[2] = input_msg.data.force.z;
        wrench.torque[0] = input_msg.data.torque.x;
        wrench.torque[1] = input_msg.data.torque.y;
        wrench.torque[2] = input_msg.data.torque.z;

        inp->setValue(wrench);
        if (make_old_data){
            input_msg.fs = OldData;
        }
    }
}

WrenchInputHandler::~WrenchInputHandler() {

};



} // namespace etasl



#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(etasl::WrenchInputHandler, etasl::InputHandler)
