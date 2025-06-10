#pragma once

#include "geometry_msgs/msg/wrench.hpp"
#include "etasl_task_utils/inputhandler.hpp"
#include <expressiongraph/context.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <unordered_map>

#include "etasl_task_utils/flowstatus.hpp"
// #include <mutex>

namespace etasl {
using namespace KDL;

class WrenchInputHandler : public InputHandler {
public:
    typedef geometry_msgs::msg::Wrench MsgType;
    typedef std::shared_ptr<WrenchInputHandler> SharedPtr;

private:
    // struct BufferElement {
    //     double most_recent_value; // most recent value obtained during the current sample period
    //     bool adapted; // the inputChannel is only updated when a value was received.
    // };
    // std::unordered_map<std::string, BufferElement> buffer;


    struct InputData {
        MsgType data;
        FlowStatus fs;
    } input_msg;

    std::string topicname;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    rclcpp::Subscription<MsgType>::SharedPtr sub;
    MsgType msg;
    MsgType default_msg;
    KDL::Wrench wrench;
    std::string when_unpublished;
    bool initialized;
    bool activated;
    std::string name;
    int nroftries;
    int counter;
    int depth;
    std::string varname;
    etasl::VariableType<KDL::Wrench>::Ptr inp;
    rclcpp::CallbackGroup::SharedPtr cbg;

public:
    WrenchInputHandler();

    virtual bool construct(
        std::string name,    
        rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
        const Json::Value& parameters,
        boost::shared_ptr<etasl::JsonChecker> jsonchecker) override;

    /**
     * will only return true if it has received values for all the joints named in jnames.
    */
    virtual bool initialize(
        Context::Ptr ctx,
        const std::vector<std::string>& jnames,
        const std::vector<std::string>& fnames,
        Eigen::VectorXd& jpos,
        Eigen::VectorXd& fpos) override;

    virtual void on_new_message(const MsgType& msg);

    virtual void update(
        double time,
        const std::vector<std::string>& jnames,
        Eigen::VectorXd& jpos,
        const std::vector<std::string>& fnames,
        Eigen::VectorXd& fpos) override;

    virtual void on_activate(Context::Ptr ctx,    
                            const std::vector<std::string>& jnames,
                            const std::vector<std::string>& fnames) override;


    virtual void on_deactivate(Context::Ptr ctx) override;

    virtual const std::string& getName() const override;

    void consume_data(const bool& make_old_data);

    virtual ~WrenchInputHandler();
};

} // namespace etasl
