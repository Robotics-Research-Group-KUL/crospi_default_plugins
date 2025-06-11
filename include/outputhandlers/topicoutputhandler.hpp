#pragma once

#include "etasl_task_utils/outputhandler.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "etasl_interfaces/msg/output.hpp"

namespace etasl {
using namespace KDL;

class TopicOutputHandler : public OutputHandler {
private:
    std::vector<Expression<double>::Ptr> outp;
    Context::Ptr ctx;
    std::vector<std::string> not_found;
    std::string topicname;
    std::vector<std::string> varnames;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    rclcpp_lifecycle::LifecyclePublisher<etasl_interfaces::msg::Output>::SharedPtr pub;
    etasl_interfaces::msg::Output msg;
    bool initialized;
    bool activated;
    std::string name;

public:
    /**
     */
    TopicOutputHandler();


    /**
     * @return get the name of this instance of TopicOutputHandler
    */
    virtual const std::string& getName() const override;

    virtual bool construct(
        std::string _name, 
        rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
        const Json::Value& _parameters,
        std::shared_ptr<etasl::JsonChecker> _jsonchecker) override;

    virtual bool initialize(
        Context::Ptr ctx,
        const std::vector<std::string>& jnames,
        const std::vector<std::string>& fnames) override;

    /**
     * updates the values for the output and prints them in a tab-delimited format
     */
    virtual void update(
        const std::vector<std::string>& jnames,
        const Eigen::VectorXd& jpos,
        const Eigen::VectorXd& jvel,
        const std::vector<std::string>& fnames,
        const Eigen::VectorXd& fvel,
        const Eigen::VectorXd& fpos) override;

    // virtual void finalize();

    virtual void on_activate(Context::Ptr ctx,    
                            const std::vector<std::string>& jnames,
                            const std::vector<std::string>& fnames) override;


    virtual void on_deactivate(Context::Ptr ctx) override;

}; // TopicOutputHandler
} // namespace KDL
