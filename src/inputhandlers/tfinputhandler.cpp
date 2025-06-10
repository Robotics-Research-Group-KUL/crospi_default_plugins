#include "inputhandlers/tfinputhandler.hpp"
// #include "rclcpp/wait_for_message.hpp"
#include <fmt/format.h>

#include <kdl/frames.hpp>

namespace etasl {


TFInputHandler::TFInputHandler()
    : frame(KDL::Vector::Zero()) //Identity rotation and zero position
    , counter(1)
    , counter_deriv(1)
    , initialized(false)
    , activated(false)
    , enable_feedforward(false)
{
    

}

bool TFInputHandler::construct(
    std::string _name,
    rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
    const Json::Value& parameters,
    boost::shared_ptr<etasl::JsonChecker> jsonchecker)
{
    name = _name;
    node = _node;
    tf_buffer = std::make_shared<tf2_ros::Buffer>(_node->get_clock(), tf2::durationFromSec(0));
    //Initialize tf_listener:
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    nroftries = jsonchecker->asInt(parameters, "number_of_tries");

    when_unpublished = jsonchecker->asString(parameters, "when_unpublished");
    target_frame = jsonchecker->asString(parameters, "child_link");
    source_frame = jsonchecker->asString(parameters, "parent_link");

    varname = jsonchecker->asString(parameters, "varname");

    if(jsonchecker->is_member(parameters, "feedforward_input_topic"))
    {
        enable_feedforward = true;
        feedforward_input_topic = jsonchecker->asString(parameters, "feedforward_input_topic");
    }


    default_msg.translation.x = jsonchecker->asDouble(parameters, "default_pose/origin/x");
    default_msg.translation.y = jsonchecker->asDouble(parameters, "default_pose/origin/y");
    default_msg.translation.z = jsonchecker->asDouble(parameters, "default_pose/origin/z");

    // cache_time = tf2::durationFromSec(jsonchecker->asDouble(parameters, "cache_time"));

    std::string type_of_orientation = "";

    for (const auto& key : parameters["default_pose"]["orientation"].getMemberNames()) {
        if (key.rfind("is-", 0) == 0) { // Check if key starts with "is-"
            type_of_orientation = key.substr(3);
        }
    }

    if (type_of_orientation == "quaternion") {
        default_msg.rotation.x = jsonchecker->asDouble(parameters, "default_pose/orientation/x");
        default_msg.rotation.y = jsonchecker->asDouble(parameters, "default_pose/orientation/y");
        default_msg.rotation.z = jsonchecker->asDouble(parameters, "default_pose/orientation/z");
        default_msg.rotation.w = jsonchecker->asDouble(parameters, "default_pose/orientation/w");
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
        default_msg.rotation.x = x;
        default_msg.rotation.y = y;
        default_msg.rotation.z = z;
        default_msg.rotation.w = w;
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

bool TFInputHandler::initialize(
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

      
    if(enable_feedforward){
        using namespace std::chrono_literals;
        using namespace std::placeholders;
        auto cb = std::bind(&TFInputHandler::on_new_message_deriv, this, _1);
        auto qos = rclcpp::SensorDataQoS().keep_last(1).lifespan(100ms);
        deriv_sub = node->create_subscription<MsgTypeDeriv>(feedforward_input_topic, qos, cb);
    
        if(!deriv_sub)
        {
            RCLCPP_FATAL(node->get_logger(), "Could not create subscriber associated to feedforward_input_topic: %s.", feedforward_input_topic.c_str());
            return false;
        }
    }

    initialized = true;
    RCLCPP_INFO(node->get_logger(), "TFinputhandler has been initialized correctly ________________________");

    return true;
}


void TFInputHandler::update(
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
    if(activated){
        fetch_tf_transform();
    }


    if(activated && input_msg.fs == NoData && when_unpublished=="throw_error"){

        if (counter >= nroftries){
            RCLCPP_ERROR(node->get_logger(), "TF transform from %s to %s could be not found after %i tries and therefore the node is shutting down. Consider changing when_unpublished policy of inputhandler %s to use_default if desired. ", source_frame.c_str(), target_frame.c_str(), nroftries, getName().c_str());
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
            RCLCPP_ERROR(node->get_logger(), "Twist could not be read from topic %s and used as feedforward for TF after %i tries and therefore the node is shutting down. Consider changing when_unpublished policy of inputhandler %s to use_default to use the default value (zero twist!) insted. ", feedforward_input_topic.c_str(), nroftries, getName().c_str());
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

void TFInputHandler::on_activate(Context::Ptr ctx,
    const std::vector<std::string>& jnames,
    const std::vector<std::string>& fnames) 
{
    // std::cout << "entering on activate =======================" << std::endl;
    if(!initialized){
        RCLCPP_WARN(node->get_logger(), "The TFInputHandler cannot be activated since it has not been initialized yet");
        return;
    }
    if(activated){
        RCLCPP_WARN(node->get_logger(), "The TFInputHandler cannot be activated since it has already been activated. Call on_deactivate first.");
        return;
    }

    fetch_tf_transform();

    inp = ctx->getInputChannel<KDL::Frame>(varname); 
    if (!inp) {
        RCLCPP_INFO(node->get_logger(), fmt::format("No Frame input channel with the name {} was created in the LUA specification, and thus the TF will not be requested. If desired, this can be created with with createInputChannelFrame function.",varname).c_str());
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

void TFInputHandler::on_deactivate(Context::Ptr ctx) {
    // std::cout << "entering on deactivate =======================" << std::endl;

    inp = nullptr;
    activated=false;
}

const std::string& TFInputHandler::getName() const
{
    return name;
}

void TFInputHandler::consume_data(const bool& make_old_data)
{
    if (inp) {
        //TODO: Replace with frame (KDL::Frame)
        frame.p[0] = input_msg.data.translation.x;
        frame.p[1] = input_msg.data.translation.y;
        frame.p[2] = input_msg.data.translation.z;
        frame.M = KDL::Rotation::Quaternion(
            input_msg.data.rotation.x,
            input_msg.data.rotation.y,
            input_msg.data.rotation.z,
            input_msg.data.rotation.w);

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

void TFInputHandler::consume_data_deriv(const bool& make_old_data)
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

void TFInputHandler::fetch_tf_transform()
{
    // if (tf_buffer.canTransform(target_frame, source_frame, tf2::TimePointZero))
    // {
        try
        {
            transform_stamped = tf_buffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero); //Not adding a fourth argument (timeout) makes the method (hopefully, according to the little and hidden documentation I found) non-blocking
            input_msg.data = transform_stamped.transform;
            input_msg.fs = NewData;
        }
        catch (const tf2::TransformException &ex)
        {
            //Include source_frame in warning:
            // RCLCPP_WARN(node->get_logger(), "Could not transform TF from %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
        }
    // }
    // else
    // {
    //     // RCLCPP_WARN(node->get_logger(), "TF transform from %s to %s not yet available", source_frame.c_str(), target_frame.c_str());
    // }

}

void TFInputHandler::on_new_message_deriv(const TFInputHandler::MsgTypeDeriv& msg)
{
    input_msg_deriv.fs = NewData;
    input_msg_deriv.data = msg;
}

TFInputHandler::~TFInputHandler() {

};



} // namespace etasl


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(etasl::TFInputHandler, etasl::InputHandler)
