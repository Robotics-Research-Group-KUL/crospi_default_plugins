#pragma once

#include "etasl_task_utils/inputhandler.hpp"
#include <expressiongraph/context.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <unordered_map>

#include "etasl_task_utils/flowstatus.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include "geometry_msgs/msg/twist.hpp"

#include <jsoncpp/json/json.h>
#include "etasl_task_utils/json_checker.hpp"




// #include <mutex>

namespace etasl {
using namespace KDL;

class TFInputHandler : public InputHandler {
public:
    typedef geometry_msgs::msg::Transform MsgType;
    typedef geometry_msgs::msg::Twist MsgTypeDeriv;
    typedef std::shared_ptr<TFInputHandler> SharedPtr;

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

    struct InputDataDeriv {
        MsgTypeDeriv data;
        FlowStatus fs;
    } input_msg_deriv;

    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    rclcpp::Subscription<MsgTypeDeriv>::SharedPtr deriv_sub;


    int nroftries;
    std::string when_unpublished;
    std::string target_frame;
    std::string source_frame;
    std::string varname;
    std::string feedforward_input_topic;
    MsgType default_msg;
    MsgTypeDeriv default_msg_deriv;

    bool enable_feedforward;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::string name;
    KDL::Frame frame;
    KDL::Twist twist;
    int time_ndx;
    int counter;
    int counter_deriv;
    bool initialized;
    bool activated;
    MsgType msg;
    etasl::VariableType<KDL::Frame>::Ptr inp;
    rclcpp::CallbackGroup::SharedPtr cbg;
    geometry_msgs::msg::TransformStamped transform_stamped;


public:
    TFInputHandler();

    virtual bool construct(
        std::string name,    
        rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
        const Json::Value& parameters,
        std::shared_ptr<etasl::JsonChecker> jsonchecker) override;

    /**
     * will only return true if it has received values for all the joints named in jnames.
    */
    virtual bool initialize(
        Context::Ptr ctx,
        const std::vector<std::string>& jnames,
        const std::vector<std::string>& fnames,
        Eigen::VectorXd& jpos,
        Eigen::VectorXd& fpos) override;


    virtual void update(
        double time,
        const std::vector<std::string>& jnames,
        Eigen::VectorXd& jpos,
        const std::vector<std::string>& fnames,
        Eigen::VectorXd& fpos) override;

    virtual void on_activate(Context::Ptr ctx,    
                            const std::vector<std::string>& jnames,
                            const std::vector<std::string>& fnames,
                            boost::shared_ptr<solver> slv) override;


    virtual void on_deactivate(Context::Ptr ctx) override;

    virtual const std::string& getName() const override;

    void consume_data(const bool& make_old_data);
    void consume_data_deriv(const bool& make_old_data);
    void fetch_tf_transform();

    void on_new_message_deriv(const TFInputHandler::MsgTypeDeriv& msg);

    virtual ~TFInputHandler();
};

} // namespace etasl
