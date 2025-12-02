//  Copyright (c) 2025 KU Leuven, Belgium
//
//  Authors: Santiago Iregui and Erwin Aertbeliën
//  emails: <santiago.iregui@kuleuven.be> and <erwin.aertbelien@kuleuven.be>
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


#pragma once

#include "crospi_utils/inputhandler.hpp"
#include <expressiongraph/context.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <unordered_map>

#include "crospi_utils/flowstatus.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include "geometry_msgs/msg/twist.hpp"

#include <jsoncpp/json/json.h>
#include "crospi_utils/json_checker.hpp"




// #include <mutex>

namespace etasl {
using namespace KDL;

class PoseInputHandler : public InputHandler {
public:
    typedef geometry_msgs::msg::Pose MsgType;
    typedef geometry_msgs::msg::Twist MsgTypeDeriv;
    typedef std::shared_ptr<PoseInputHandler> SharedPtr;

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
    rclcpp::Subscription<MsgType>::SharedPtr sub;
    rclcpp::Subscription<MsgTypeDeriv>::SharedPtr deriv_sub;


    int nroftries;
    std::string when_unpublished;

    std::string varname;
    std::string feedforward_input_topic;
    MsgType default_msg;
    MsgTypeDeriv default_msg_deriv;

    bool enable_feedforward;
    std::string name;
    std::string topicname;
    KDL::Frame frame;
    KDL::Twist twist;
    int time_ndx;
    int counter;
    int depth;
    int counter_deriv;
    bool initialized;
    bool activated;
    MsgType msg;
    etasl::VariableType<KDL::Frame>::Ptr inp;
    rclcpp::CallbackGroup::SharedPtr cbg;


public:
    PoseInputHandler();

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

    void on_new_message(const MsgType& msg);


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

    void on_new_message_deriv(const PoseInputHandler::MsgTypeDeriv& msg);

    virtual ~PoseInputHandler();
};

} // namespace etasl
