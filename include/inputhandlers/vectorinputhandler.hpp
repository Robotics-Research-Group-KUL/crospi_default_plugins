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
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "geometry_msgs/msg/twist.hpp"

#include <jsoncpp/json/json.h>
#include "crospi_utils/json_checker.hpp"




// #include <mutex>

namespace etasl {
using namespace KDL;

class VectorInputHandler : public InputHandler {
public:
    typedef geometry_msgs::msg::Vector3 MsgType_vector;
    typedef geometry_msgs::msg::Point MsgType_point;
    typedef geometry_msgs::msg::Vector3 MsgTypeDeriv;
    typedef std::shared_ptr<VectorInputHandler> SharedPtr;

private:
    // struct BufferElement {
    //     double most_recent_value; // most recent value obtained during the current sample period
    //     bool adapted; // the inputChannel is only updated when a value was received.
    // };
    // std::unordered_map<std::string, BufferElement> buffer;


    struct InputData {
        MsgType_vector data;
        FlowStatus fs;
    } input_msg;

    struct InputDataDeriv {
        MsgTypeDeriv data;
        FlowStatus fs;
    } input_msg_deriv;

    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    rclcpp::Subscription<MsgType_vector>::SharedPtr sub_vector;
    rclcpp::Subscription<MsgType_point>::SharedPtr sub_point;
    rclcpp::Subscription<MsgTypeDeriv>::SharedPtr deriv_sub;


    int nroftries;
    std::string when_unpublished;
    std::string msg_type;

    std::string varname;
    std::string feedforward_input_topic;
    MsgType_vector default_msg;
    MsgTypeDeriv default_msg_deriv;

    bool enable_feedforward;
    std::string name;
    std::string topicname;
    KDL::Vector kdl_vector;
    KDL::Vector kdl_velocity;
    int time_ndx;
    int counter;
    int depth;
    int counter_deriv;
    bool initialized;
    bool activated;
    MsgType_vector msg;
    etasl::VariableType<KDL::Vector>::Ptr inp;
    rclcpp::CallbackGroup::SharedPtr cbg;


public:
    VectorInputHandler();

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

    void on_new_message_vector(const MsgType_vector& msg);
    void on_new_message_point(const MsgType_point& msg);


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

    void on_new_message_deriv(const VectorInputHandler::MsgTypeDeriv& msg);

    virtual ~VectorInputHandler();
};

} // namespace etasl
