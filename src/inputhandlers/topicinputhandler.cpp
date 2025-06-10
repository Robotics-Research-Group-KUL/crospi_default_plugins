#include "inputhandlers/topicinputhandler.hpp"
#include "etasl_task_utils/etasl_error.hpp"
#include "etasl_task_utils/outputhandler.hpp" // to get to cut_global
#include <fmt/format.h>

namespace etasl {

TopicInputHandler::TopicInputHandler()
{
    
}

bool TopicInputHandler::construct(
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

const std::string& TopicInputHandler::getName() const
{
    return name;
}

bool TopicInputHandler::initialize(
    Context::Ptr ctx,
    const std::vector<std::string>& jnames,
    const std::vector<std::string>& fnames,
    Eigen::VectorXd& jpos,
    Eigen::VectorXd& fpos)
{
    using namespace std::placeholders;
    size_t ndx = 0;
    for (auto v : ctx->input_vars) {
        auto p = ctx->getInputChannel<double>(v.first);
        auto name = cut_global(v.first);
        if (p != nullptr) {
            RCLCPP_INFO(node->get_logger(), "      input channel: %s",name.c_str());
            Buffer buf;
            buf.adapted = false;
            buf.most_recent_value = 0.0;
            buf.inputChannel = p;
            buffer.push_back(buf);
            inputs[name] = ndx;
            ndx++;
        }
    }

    sub = node->create_subscription<etasl_interfaces::msg::Input>(topicname, 10,
        std::bind(&TopicInputHandler::on_new_message, this, _1));

    if (!sub) {
        RCLCPP_FATAL(node->get_logger(), "Could not create subscriber associated to the topic input handler for topic '%s'.", topicname.c_str());
        return false;
    }

    return true;
}

void TopicInputHandler::on_new_message(const etasl_interfaces::msg::Input& msg)
{
    if (msg.data.size() != msg.names.size()) {
        throw etasl_error(etasl_error::INPUTHANDLER_INCONSISTENT_MESSAGE, "message with different lengths for data and names (topic '" + topicname + "')");
    }
    for (size_t i = 0; i < msg.names.size(); ++i) {
        auto p = inputs.find(msg.names[i]);
        if (p == inputs.end()) {

            throw etasl_error(
                etasl_error::INPUTHANDLER_INCONSISTENT_MESSAGE,
                fmt::format("message with unknown name '{}' for input channel for topic '{}')", msg.names[i], topicname));
        }
        size_t ndx = p->second;
        buffer[ndx].adapted = true;
        buffer[ndx].most_recent_value = msg.data[i];
    }
}

void TopicInputHandler::update(
    double time, 
    const std::vector<std::string>& jnames,
    Eigen::VectorXd& jpos, 
    const std::vector<std::string>& fnames,
    Eigen::VectorXd& fpos)
{
    for (auto& b : buffer) {
        if (b.adapted) {
            b.adapted = false;
            std::cout << "setValue " << b.most_recent_value << std::endl;
            b.inputChannel->setValue(b.most_recent_value);
        }
    }
}


} // namespace etasl

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(etasl::TopicInputHandler, etasl::InputHandler)
