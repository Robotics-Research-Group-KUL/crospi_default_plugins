#include "inputhandlers/twistinputhandler.hpp"
// #include "rclcpp/wait_for_message.hpp"
#include <fmt/format.h>

namespace etasl {


TwistInputHandler::TwistInputHandler()
    : twist(KDL::Twist::Zero())
    , counter(0)
    , initialized(false)
    , activated(false)
{
    //Constructor with no arguments needed for plugin registration
}


bool TwistInputHandler::construct(
    std::string _name,
    rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
    const Json::Value& parameters,
    boost::shared_ptr<etasl::JsonChecker> jsonchecker)
{
    node = _node;
    topicname = jsonchecker->asString(parameters, "topic-name");

    name = fmt::format("{}({})",_name, topicname);
        
    depth = jsonchecker->asInt(parameters, "depth");
    nroftries = jsonchecker->asInt(parameters, "number_of_tries");

    when_unpublished = jsonchecker->asString(parameters, "when_unpublished");

    varname = jsonchecker->asString(parameters, "varname");


    default_msg.linear.x = jsonchecker->asDouble(parameters, "default_twist/linear/x");
    default_msg.linear.y = jsonchecker->asDouble(parameters, "default_twist/linear/y");
    default_msg.linear.z = jsonchecker->asDouble(parameters, "default_twist/linear/z");
    default_msg.angular.x = jsonchecker->asDouble(parameters, "default_twist/angular/x");
    default_msg.angular.y = jsonchecker->asDouble(parameters, "default_twist/angular/y");
    default_msg.angular.z = jsonchecker->asDouble(parameters, "default_twist/angular/z");

    // cbg = node->create_callback_group()
    // sub = node->create_subscription<MsgType>(_topic_name, rclcpp::QoS(10), cb);

    input_msg.fs = NoData;

    return true;
    
}

bool TwistInputHandler::initialize(
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
    auto cb = std::bind(&TwistInputHandler::on_new_message, this, _1);
    auto qos = rclcpp::SensorDataQoS().keep_last(depth).lifespan(100ms);
    sub = node->create_subscription<MsgType>(topicname, qos, cb);

    if (!sub) {
        RCLCPP_FATAL(node->get_logger(), "Could not create subscriber for input handler '%s'.", name.c_str());
        return false;
    }

    initialized = true;
    RCLCPP_INFO(node->get_logger(), "twistinputhandler has been initialized correctly ________________________");
    return true;
}

void TwistInputHandler::on_new_message(const TwistInputHandler::MsgType& msg)
{
    input_msg.fs = NewData;
    input_msg.data = msg;
}

void TwistInputHandler::update(
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
            RCLCPP_ERROR(node->get_logger(), fmt::format("twistinputhandler could not initialize since no data has been published in topic {} and therefore the node is shutting down. Consider changing when_unpublished policy of inputhandler to use_default if desired.",name).c_str());
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

void TwistInputHandler::on_activate(Context::Ptr ctx,
    const std::vector<std::string>& jnames,
    const std::vector<std::string>& fnames) 
{
    // std::cout << "entering on activate =======================" << std::endl;
    if(!initialized){
        RCLCPP_WARN(node->get_logger(), "The TwistInputHandler cannot be activated since it has not been initialized yet");
        return;
    }
    if(activated){
        RCLCPP_WARN(node->get_logger(), "The TwistInputHandler cannot be activated since it has already been activated. Call on_deactivate first.");
        return;
    }

    inp = ctx->getInputChannel<KDL::Twist>(varname); 
    if (!inp) {
        RCLCPP_INFO(node->get_logger(), fmt::format("No twist input channel with the name {} was created in the LUA specification, and thus topic {} will be ignored. If desired, this can be created with with createInputChannelTwist function.",varname, name).c_str());
        activated = false;
    } 
    else {
        // inps->setJacobian(time_ndx, default_value_kdl );
        consume_data(false); 
        activated = true;
    }
}

void TwistInputHandler::on_deactivate(Context::Ptr ctx) {
    // std::cout << "entering on deactivate =======================" << std::endl;

    inp = nullptr;
    activated=false;
}

const std::string& TwistInputHandler::getName() const
{
    return name;
}

void TwistInputHandler::consume_data(const bool& make_old_data)
{
    if (inp) {
        twist.vel[0] = input_msg.data.linear.x;
        twist.vel[1] = input_msg.data.linear.y;
        twist.vel[2] = input_msg.data.linear.z;
        twist.rot[0] = input_msg.data.angular.x;
        twist.rot[1] = input_msg.data.angular.y;
        twist.rot[2] = input_msg.data.angular.z;

        inp->setValue(twist);
        if (make_old_data){
            input_msg.fs = OldData;
        }
    }
}

TwistInputHandler::~TwistInputHandler() {

};



} // namespace etasl


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(etasl::TwistInputHandler, etasl::InputHandler)
