#include "outputhandlers/jointstateoutputhandler.hpp"
#include "rclcpp/rclcpp.hpp"
#include <fmt/format.h>

namespace etasl {

using namespace std::chrono_literals;

JointStateOutputHandler::JointStateOutputHandler()
    : initialized(false)
    , activated(false)
{

}


bool JointStateOutputHandler::construct(
    std::string _name, 
    rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
    const Json::Value& parameters,
    boost::shared_ptr<etasl::JsonChecker> jsonchecker)
{
    node = _node;
    topicname = jsonchecker->asString(parameters, "topic-name");
    name = fmt::format("{}({})",_name, topicname);

    return true;
}

bool JointStateOutputHandler::initialize(
    Context::Ptr ctx,
    const std::vector<std::string>& jnames,
    const std::vector<std::string>& fnames)
{
    // std::cout << "entering initialize JointStateOutputHandler =======================" << std::endl;

    if(!initialized){
        // pub = node->create_publisher<MsgType>(topicname, rclcpp::SensorDataQoS());
        // auto qos = rclcpp::SensorDataQoS().keep_last(1).lifespan(100ms).reliability(rclcpp::ReliabilityPolicy::Reliable);
        auto qos = rclcpp::SensorDataQoS().keep_last(1).lifespan(100ms);
        pub = node->create_publisher<MsgType>(topicname, qos);
        pub->on_deactivate();
        initialized = true;
        std::cout << "initialized JointStateOutputHandler=======================" << std::endl;
    }
    else{
        RCLCPP_WARN(node->get_logger(), "Ignoring request: the jointstateoutputhandler was already initialized, so it cannot be initialized again.");
        return false;
    }
    return true;
}

void JointStateOutputHandler::update(
    const std::vector<std::string>& jnames,
    const Eigen::VectorXd& jpos,
    const Eigen::VectorXd& jvel,
    const std::vector<std::string>& fnames,
    const Eigen::VectorXd& fvel,
    const Eigen::VectorXd& fpos)
{
        // std::cout << "entering on update =======================" << std::endl;

    if(!activated){
        // RCLCPP_WARN(node->get_logger(), "The jointstateoutputhandler cannot be updated since it has not been initialized yet");
        return;
    }
    assert(msg.position.size() == jnames.size() /* size of msg.position and jnames vector is not the same */); 
    assert(msg.velocity.size() == jnames.size() /* size of msg.velocity and jnames vector is not the same */); 


    for (size_t i = 0; i < jnames.size(); ++i) {
        msg.position[i] = jpos[i];
        msg.velocity[i] = jvel[i];
    }
    // msg.header.set__stamp()
    // RCLCPP_INFO(node->get_logger(), "JointStateOutputHandler message: ");
    // RCLCPP_INFO(node->get_logger(), fmt::format("{:16}", fmt::join(jnames.begin(), jnames.end(), ", ")).c_str());
    // RCLCPP_INFO(node->get_logger(), fmt::format("{:16.5}", fmt::join(jpos.begin(), jpos.end(), ", ") ).c_str());
    // RCLCPP_INFO(node->get_logger(), fmt::format("{:16.5}", fmt::join(jvel.begin(), jvel.end(), ", ") ).c_str());
    // msg.header.stamp = node->get_clock()->now();
    msg.header.stamp = node->now();
    pub->publish(msg);
}


void JointStateOutputHandler::on_activate(Context::Ptr ctx,
    const std::vector<std::string>& jnames,
    const std::vector<std::string>& fnames) 
{
    // std::cout << "entering on activate =======================" << std::endl;
    if(!initialized){
        RCLCPP_WARN(node->get_logger(), "The jointstateoutputhandler cannot be activated since it has not been initialized yet");
        return;
    }
    if(activated){
        RCLCPP_WARN(node->get_logger(), "The jointstateoutputhandler cannot be activated since it has already been activated. Call on_deactivate first.");
        return;
    }
    for (auto n : jnames) {
        msg.name.push_back(cut_global(n));
        msg.position.push_back(0.0);
        msg.velocity.push_back(0.0);
    }
    assert(msg.position.size() == jnames.size() /* size of msg.position and jnames vector is not the same */); 
    assert(msg.velocity.size() == jnames.size() /* size of msg.velocity and jnames vector is not the same */); 

    pub->on_activate();
    activated = true;
}

void JointStateOutputHandler::on_deactivate(Context::Ptr ctx) {
    // std::cout << "entering on deactivate =======================" << std::endl;
   if(initialized){
        for (size_t i = 0; i < msg.velocity.size(); ++i) {
            msg.velocity[i] = 0.0;
        }
        msg.header.stamp = node->get_clock()->now();
        pub->publish(msg);

        pub->on_deactivate();
    }
    msg.name.clear();
    msg.position.clear();
    msg.velocity.clear();
    activated=false;
   
}

void JointStateOutputHandler::finalize()

{
        // std::cout << "entering on finalize =======================" << std::endl;
   if(initialized){
        for (size_t i = 0; i < msg.velocity.size(); ++i) {
            msg.velocity[i] = 0.0;
        }
        msg.header.stamp = node->get_clock()->now();
        // pub->on_activate();
        pub->publish(msg);        
        // Example that uses wait_for_all_acked: https://github.com/ros2/examples/pull/316/files
        pub->wait_for_all_acked(std::chrono::milliseconds(500)); //Blocks until all subscribers have gotten the message with a timeout
        // pub->wait_for_all_acked(); //Waits until all subscribers have gotten the message without a timeout
        pub->on_deactivate();
    }
    msg.name.clear();
    msg.position.clear();
    msg.velocity.clear();
    activated=false;
}

const std::string& JointStateOutputHandler::getName() const
{
    return name;
}

} // namespace etasl


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(etasl::JointStateOutputHandler, etasl::OutputHandler)
