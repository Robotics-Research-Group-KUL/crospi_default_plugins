#include "inputhandlers/poseinputhandler.hpp"
// #include "rclcpp/wait_for_message.hpp"
#include <fmt/format.h>

#include <kdl/frames.hpp>

namespace etasl {


PoseInputHandler::PoseInputHandler()
    : frame(KDL::Vector::Zero()) //Identity rotation and zero position
    , counter(1)
    , counter_deriv(1)
    , initialized(false)
    , activated(false)
    , enable_feedforward(false)
{
    

}

bool PoseInputHandler::construct(
    std::string _name,
    rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
    const Json::Value& parameters,
    std::shared_ptr<etasl::JsonChecker> jsonchecker)
{
    
    node = _node;
    
    topicname = jsonchecker->asString(parameters, "topic-name");
    name = fmt::format("{}({})",_name, topicname);
    depth = jsonchecker->asInt(parameters, "depth");

    nroftries = jsonchecker->asInt(parameters, "number_of_tries");

    when_unpublished = jsonchecker->asString(parameters, "when_unpublished");


    varname = jsonchecker->asString(parameters, "varname");

    if(jsonchecker->is_member(parameters, "feedforward_input_topic"))
    {
        enable_feedforward = true;
        feedforward_input_topic = jsonchecker->asString(parameters, "feedforward_input_topic");
    }


    default_msg.position.x = jsonchecker->asDouble(parameters, "default_pose/origin/x");
    default_msg.position.y = jsonchecker->asDouble(parameters, "default_pose/origin/y");
    default_msg.position.z = jsonchecker->asDouble(parameters, "default_pose/origin/z");


    std::string type_of_orientation = "";

    for (const auto& key : parameters["default_pose"]["orientation"].getMemberNames()) {
        if (key.rfind("is-", 0) == 0) { // Check if key starts with "is-"
            type_of_orientation = key.substr(3);
        }
    }

    if (type_of_orientation == "quaternion") {
        default_msg.orientation.x = jsonchecker->asDouble(parameters, "default_pose/orientation/x");
        default_msg.orientation.y = jsonchecker->asDouble(parameters, "default_pose/orientation/y");
        default_msg.orientation.z = jsonchecker->asDouble(parameters, "default_pose/orientation/z");
        default_msg.orientation.w = jsonchecker->asDouble(parameters, "default_pose/orientation/w");
        // RCLCPP_INFO(node->get_logger(), "Type of orientation: quaternion!!!!!!!!!!!!!");
        
    }
    else if (type_of_orientation == "rpy") {
        KDL::Rotation rot = KDL::Rotation::RPY(
            jsonchecker->asDouble(parameters, "default_pose/orientation/roll"),
            jsonchecker->asDouble(parameters, "default_pose/orientation/pitch"),
            jsonchecker->asDouble(parameters, "default_pose/orientation/yaw")
        );
        double x, y, z, w;
        rot.GetQuaternion(x, y, z, w);
        default_msg.orientation.x = x;
        default_msg.orientation.y = y;
        default_msg.orientation.z = z;
        default_msg.orientation.w = w;
        // RCLCPP_INFO(node->get_logger(), "Type of orientation: rpy!!!!!!!!!!!!!");
    }
    else{
        RCLCPP_ERROR(node->get_logger(), "Type of orientation %s is not valid. It should be either 'quaternion' or 'rpy'.", type_of_orientation.c_str());
        return false;
    }

    input_msg.fs = NoData;
    input_msg.data = default_msg;
    // transform_stamped = default_msg;

    //initialize input_msg_deriv with a zero twist msg:
    default_msg_deriv.linear.x = 0.0;
    default_msg_deriv.linear.y = 0.0;
    default_msg_deriv.linear.z = 0.0;
    default_msg_deriv.angular.x = 0.0;
    default_msg_deriv.angular.y = 0.0;
    default_msg_deriv.angular.z = 0.0;

    input_msg_deriv.data = default_msg_deriv;
    input_msg_deriv.fs = NoData;

    return true;
}

bool PoseInputHandler::initialize(
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

    if(input_msg_deriv.fs == NoData && when_unpublished=="use_default"){
        input_msg_deriv.fs = NewData; //Fakes new Data 
        input_msg_deriv.data = default_msg_deriv;

    }
    else if(input_msg_deriv.fs == NoData && when_unpublished=="throw_error"){
        input_msg_deriv.data = default_msg_deriv;
    }

    using namespace std::chrono_literals;
    using namespace std::placeholders;
    auto cb = std::bind(&PoseInputHandler::on_new_message, this, _1);
    auto qos = rclcpp::SensorDataQoS().keep_last(depth).lifespan(100ms);
    
    sub = node->create_subscription<MsgType>(topicname, qos, cb);

    if (!sub) {
        RCLCPP_FATAL(node->get_logger(), "Could not create subscriber for input handler '%s'.", name.c_str());
        return false;
    }

      
    if(enable_feedforward){
        using namespace std::chrono_literals;
        using namespace std::placeholders;
        auto cb = std::bind(&PoseInputHandler::on_new_message_deriv, this, _1);
        auto qos = rclcpp::SensorDataQoS().keep_last(1).lifespan(100ms);
        deriv_sub = node->create_subscription<MsgTypeDeriv>(feedforward_input_topic, qos, cb);
    
        if(!deriv_sub)
        {
            RCLCPP_FATAL(node->get_logger(), "Could not create subscriber associated to feedforward_input_topic: %s.", feedforward_input_topic.c_str());
            return false;
        }
    }

    initialized = true;
    RCLCPP_INFO(node->get_logger(), "Poseinputhandler has been initialized correctly ________________________");

    return true;
}


void PoseInputHandler::update(
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
    // input_msg.data = msg;



    if(activated && input_msg.fs == NoData && when_unpublished=="throw_error"){

        if (counter >= nroftries){
            RCLCPP_ERROR(node->get_logger(), "Pose could be not found after %i tries and therefore the node is shutting down. Consider changing when_unpublished policy of inputhandler %s to use_default if desired. ", nroftries, getName().c_str());
            rclcpp::shutdown();
        }
        else{
            counter++;
            // std::cout << "Counter +++++: " << counter<< std::endl;
        }
        return;
    }
    else if(activated && input_msg.fs == NewData){
        consume_data(true); 
    }

    if(enable_feedforward && activated && input_msg_deriv.fs == NoData && when_unpublished=="throw_error"){

        if (counter_deriv >= nroftries){
            RCLCPP_ERROR(node->get_logger(), "Twist could not be read from topic %s and used as feedforward for Pose after %i tries and therefore the node is shutting down. Consider changing when_unpublished policy of inputhandler %s to use_default to use the default value (zero twist!) insted. ", feedforward_input_topic.c_str(), nroftries, getName().c_str());
            rclcpp::shutdown();
        }
        else{
            counter_deriv++;
        }
        return;
    }
    else if(activated && input_msg_deriv.fs == NewData){
        consume_data_deriv(true); 
    }
    // RCLCPP_INFO(node->get_logger(), "JointStateInputHander::update()");
}

void PoseInputHandler::on_activate(Context::Ptr ctx,
    const std::vector<std::string>& jnames,
    const std::vector<std::string>& fnames,
    boost::shared_ptr<solver> slv) 
{
    // std::cout << "entering on activate =======================" << std::endl;
    if(!initialized){
        RCLCPP_WARN(node->get_logger(), "The PoseInputHandler cannot be activated since it has not been initialized yet");
        return;
    }
    if(activated){
        RCLCPP_WARN(node->get_logger(), "The PoseInputHandler cannot be activated since it has already been activated. Call on_deactivate first.");
        return;
    }


    inp = ctx->getInputChannel<KDL::Frame>(varname); 
    if (!inp) {
        RCLCPP_INFO(node->get_logger(), fmt::format("No Frame input channel with the name {} was created in the LUA specification, and thus the Pose will not be acquired. If desired, this can be created with with createInputChannelFrame function.",varname).c_str());
        activated = false;
    } 
    else {
        // inps->setJacobian(time_ndx, default_value_kdl );
        time_ndx = ctx->getScalarNdx("time");
        consume_data(false); 
        consume_data_deriv(false); 
        activated = true;
    }
}

void PoseInputHandler::on_deactivate(Context::Ptr ctx) {
    // std::cout << "entering on deactivate =======================" << std::endl;

    inp = nullptr;
    activated=false;
}

const std::string& PoseInputHandler::getName() const
{
    return name;
}

void PoseInputHandler::consume_data(const bool& make_old_data)
{
    if (inp) {
        //TODO: Replace with frame (KDL::Frame)
        frame.p[0] = input_msg.data.position.x;
        frame.p[1] = input_msg.data.position.y;
        frame.p[2] = input_msg.data.position.z;
        frame.M = KDL::Rotation::Quaternion(input_msg.data.orientation.x,
                                            input_msg.data.orientation.y,
                                            input_msg.data.orientation.z,
                                            input_msg.data.orientation.w);

        // twist.vel[0] = input_msg.data.linear.x;
        // twist.vel[1] = input_msg.data.linear.y;
        // twist.vel[2] = input_msg.data.linear.z;
        // twist.rot[0] = input_msg.data.angular.x;
        // twist.rot[1] = input_msg.data.angular.y;
        // twist.rot[2] = input_msg.data.angular.z;

        // inp->setValue(twist);
        // if (make_old_data){
        //     input_msg.fs = OldData;
        // }
        inp->setValue(frame);
        if (make_old_data){
            input_msg.fs = OldData;
        }
    }
}

void PoseInputHandler::consume_data_deriv(const bool& make_old_data)
{
    if (inp) {
        twist.vel[0] = input_msg_deriv.data.linear.x;
        twist.vel[1] = input_msg_deriv.data.linear.y;
        twist.vel[2] = input_msg_deriv.data.linear.z;
        twist.rot[0] = input_msg_deriv.data.angular.x;
        twist.rot[1] = input_msg_deriv.data.angular.y;
        twist.rot[2] = input_msg_deriv.data.angular.z;

        inp->setJacobian(time_ndx, twist );
        if (make_old_data){
            input_msg_deriv.fs = OldData;
        }
    }
}


void PoseInputHandler::on_new_message(const PoseInputHandler::MsgType& msg)
{
    input_msg.fs = NewData;
    input_msg.data = msg;
}

void PoseInputHandler::on_new_message_deriv(const PoseInputHandler::MsgTypeDeriv& msg)
{
    input_msg_deriv.fs = NewData;
    input_msg_deriv.data = msg;
}

PoseInputHandler::~PoseInputHandler() {

};



} // namespace etasl


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(etasl::PoseInputHandler, etasl::InputHandler)
