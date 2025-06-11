#include "outputhandlers/topicoutputhandler.hpp"

namespace etasl {

TopicOutputHandler::TopicOutputHandler()
    : initialized(false)
    , activated(false)
{

}


bool TopicOutputHandler::construct(
    std::string _name, 
    rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
    const Json::Value& parameters,
    std::shared_ptr<etasl::JsonChecker> jsonchecker)
{
    node = _node;
    topicname = jsonchecker->asString(parameters, "topic-name");
    name = fmt::format("{}({})",_name, topicname);
    for (auto n : parameters["variable-names"]) {
    // for (auto n : jsonchecker->asArray(parameters, "variable-names")) {
        varnames.push_back(n.asString());
        // varnames.push_back(jsonchecker->asString(n, ""));
    }

    return true;
}



bool TopicOutputHandler::initialize(
    Context::Ptr ctx,
    const std::vector<std::string>& jnames,
    const std::vector<std::string>& fnames)
{
    // std::cout << "entering on initialize topicoutputhandler =======================" << std::endl;

    if(!initialized){
        pub = node->create_publisher<etasl_interfaces::msg::Output>(topicname, 10);
        pub->on_deactivate();
        initialized = true;
        // std::cout << "initialized topicoutputhandler=======================" << std::endl;
    }
    else{
        RCLCPP_WARN(node->get_logger(), "Ignoring request: the topicoutputhandler was already initialized, so it cannot be initialized again.");
        return false;
    }
    return true;
}

void TopicOutputHandler::update(
    const std::vector<std::string>& jnames,
    const Eigen::VectorXd& jpos,
    const Eigen::VectorXd& jvel,
    const std::vector<std::string>& fnames,
    const Eigen::VectorXd& fvel,
    const Eigen::VectorXd& fpos)
{
    if(!activated){
        // RCLCPP_WARN(node->get_logger(), "The topicoutputhandler cannot be updated since it has not been initialized yet");
        return;
    }
    assert( msg.data.size() == outp.size() /* size of msg.data and outp vector containing expressions is not the same */); 

    unsigned int L = outp.size();
    for (unsigned int i = 0; i < L; ++i) {
        msg.data[i] = outp[i]->value();
    }
    pub->publish(msg);
}

void TopicOutputHandler::on_activate(Context::Ptr ctx, const std::vector<std::string>& jnames, const std::vector<std::string>& fnames) {
    // std::cout << "entering on activate =======================" << std::endl;
    // if (outp.size()>0){
    //     throw std::runtime_error("Topicoutputhandler: Before calling on_activate again, on_deactivate must be called ");
    //     return;
    // }

    if(!initialized){
        RCLCPP_WARN(node->get_logger(), "The topicoutputhandler cannot be activated since it has not been initialized yet");
        return;
    }
    if(activated){
        RCLCPP_WARN(node->get_logger(), "The topicoutputhandler cannot be activated since it has already been activated. Call on_deactivate first.");
        return;
    }

    if (varnames.size() == 0) {
        // std::cout << "topic output hello1............................" << std::endl;
        for (auto ov : ctx->output_vars) {
            auto ptr = ctx->getOutputExpression<double>(ov.first);
            // std::cout << "topic output hello4444............................" << ptr->value() << std::endl;
            if (ptr != 0) {
                msg.names.push_back(cut_global(ov.first));
                outp.push_back(ptr);
                msg.data.push_back(0.0);
                msg.is_declared.push_back(true);
            }
        }
    } else {
        // msg.names = varnames;
        // msg.data.resize(varnames.size());
        for (auto i = 0; i < varnames.size(); ++i) {
            auto ptr = ctx->getOutputExpression<double>(varnames[i]);
            if (ptr != 0) {
            // std::cout << "topic output hello1222............................" << varnames[i] << std::endl;
                // std::cout << varnames[i] << "value:" << ptr->value() << std::endl;

                msg.names.push_back(cut_global(varnames[i]));
                outp.push_back(ptr);
                msg.data.push_back(0.0);
                msg.is_declared.push_back(true);
            }
            else{
                // std::cout << "topic output hello3333............................" << varnames[i] << std::endl;
                not_found.push_back(varnames[i]);
                msg.names.push_back(cut_global(varnames[i]));
                outp.push_back(Constant(0.0));
                msg.data.push_back(0.0);
                msg.is_declared.push_back(false);
            }
        }
    }
    pub->on_activate(); //This works because we check for initialized above
    activated = true;
}

void TopicOutputHandler::on_deactivate(Context::Ptr ctx) {
    outp.clear();
    msg.data.clear();
    msg.names.clear();
    msg.is_declared.clear();
    if(initialized){
        pub->on_deactivate();
    }
    activated = false;
}

const std::string& TopicOutputHandler::getName() const {
    return name;
}

// void TopicOutputHandler::finalize()
// {
//     pub.reset();
// }

} // namespace etasl

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(etasl::TopicOutputHandler, etasl::OutputHandler)