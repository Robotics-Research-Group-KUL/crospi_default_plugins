#ifndef JOINTSTATEOUTPUTHANDLER_HPP_34DF43
#define JOINTSTATEOUTPUTHANDLER_HPP_34DF43
#include "expressiongraph/context.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "etasl_task_utils/outputhandler.hpp"
#include "etasl_task_utils/etasl_error.hpp"

namespace etasl {
using namespace KDL;
/**
 * Example of a class that handles output suitable for simple, unbuffered, csv output.
 * It gets the output values from the context and outputs the results in a
 * tab-delimited file with a header containing the names of the signals
 *
 * Only handles double-expressions !
 *
 * This class is just an example, you can choose the structure of the class and its methods
 * yourself.
 */
class JointStateOutputHandler : public OutputHandler {
    typedef sensor_msgs::msg::JointState MsgType;

private:
    Context::Ptr ctx;
    std::string topicname;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    rclcpp_lifecycle::LifecyclePublisher<MsgType>::SharedPtr pub;
    MsgType msg;
    bool initialized;
    bool activated;
    std::string name;

public:
    /**
     * Gets all position and velocity values for the robot variables
     * in the specification.
     */
    JointStateOutputHandler();

    /**
     * @return a name of this instance of JointStateOutputHandler
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

    virtual void finalize() override;


    virtual void on_activate(Context::Ptr ctx,    
                            const std::vector<std::string>& jnames,
                            const std::vector<std::string>& fnames) override;
                            
    virtual void on_deactivate(Context::Ptr ctx) override;

    
}; // JointStateOutputHandler
} // namespace KDL

#endif
