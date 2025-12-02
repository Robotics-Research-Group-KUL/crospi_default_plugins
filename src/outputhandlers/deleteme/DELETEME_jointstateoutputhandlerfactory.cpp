#include "crospi_node_utils/jointstateoutputhandler.hpp"
#include "crospi_utils/etasl_error.hpp"
#include "crospi_utils/outputhandlerfactory.hpp"
#include "crospi_utils/registry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <jsoncpp/json/json.h>

namespace etasl {

/**
 * This is a factory that can create JointStateOutputHandlers
 * The platform specific part is given with the constructor of this factory
 * Afterwards, everything is generic and independent of the platform
 */
class JointStateOutputHandlerFactory : public OutputHandlerFactory {
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;

public:
    typedef std::shared_ptr<OutputHandlerFactory> SharedPtr;

    JointStateOutputHandlerFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr _node)
        : node(_node)
    {
    }

    /**
     * @brief gets the schema for the parameters of this factory
     * @return JSON schema
     */
    virtual Json::Value getSchema()
    {
        std::string schema_src = R"(
                    {
                        "$id": "JointStateOutputHandler.json",
                        "type" : "object",
                        "title":"Parameters for JointStateOutputHandler",
                        "description" : "",
                        "properties" : {
                            "is-jointstateoutputhandler" : {
                                "type":"boolean"
                            },                            
                            "topic-name" : {
                                "type":"string"
                            }
                        },
                        "required": ["is-jointstateoutputhandler","topic-name"],
                        "additionalProperties" : false
                    }
                )";
        Json::Value schema;
        Json::Reader reader;
        reader.parse(schema_src, schema);
        return schema;
    }

    /**
     * @brief gets the name of this solver
     */
    virtual const char* getName()
    {
        return "jointstateoutputhandler";
    }

    /**
     * @brief create the solver with the given parameters
     *
     */
    virtual JointStateOutputHandler::SharedPtr create(const Json::Value& parameters, std::shared_ptr<etasl::JsonChecker> jsonchecker)
    {
        std::string topic_name = jsonchecker->asString(parameters, "topic-name");
        return std::make_shared<JointStateOutputHandler>(
            node,
            topic_name);
    }

    virtual ~JointStateOutputHandlerFactory() { }
};

void registerJointStateOutputHandlerFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr _node)
{
    // be sure to use the BASE CLASS as template parameter for the Registry!
    Registry<OutputHandlerFactory>::registerFactory(std::make_shared<JointStateOutputHandlerFactory>(_node));
}

} // namespace