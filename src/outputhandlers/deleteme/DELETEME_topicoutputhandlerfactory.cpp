#include "etasl_task_utils/outputhandlerfactory.hpp"
#include "etasl_node_utils/topicoutputhandler.hpp"
#include "etasl_task_utils/etasl_error.hpp"
#include "etasl_task_utils/registry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <jsoncpp/json/json.h>

namespace etasl {


/**
 * This is a factory that can create TopicOutputHandlers
 * The platform specific part is given with the constructor of this factory
 * Afterwards, everything is generic and independent of the platform
*/
class TopicOutputHandlerFactory : public OutputHandlerFactory {
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;

public:
    typedef std::shared_ptr<OutputHandlerFactory> SharedPtr;

    TopicOutputHandlerFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr _node)
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
                        "$id": "topicoutputhandler.json",
                        "type" : "object",
                        "title":"Parameters for TopicOutputHandler",
                        "description" : "",
                        "properties" : {
                            "is-topicoutputhandler" : {
                                "type":"boolean"
                            },                            
                            "topic-name" : {
                                "type":"string"
                            },
                            "variable-names": {
                                "type":"array",
                                "items": {
                                    "type":"string"
                                }
                            }
                        },
                        "required": ["is-topicoutputhandler","topic-name","variable-names"],
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
        return "topicoutputhandler";
    }

    /**
     * @brief create the solver with the given parameters
     *
     */
    virtual TopicOutputHandler::SharedPtr create(const Json::Value& parameters, boost::shared_ptr<etasl::JsonChecker> jsonchecker)
    {
        std::string topic_name = jsonchecker->asString(parameters, "topic-name");
        std::vector<std::string> varnames;
        // for (auto n : parameters["variable-names"]) {
        for (auto n : jsonchecker->asArray(parameters, "variable-names")) {
            varnames.push_back(jsonchecker->asString(n, ""));
        }
        return std::make_shared<TopicOutputHandler>(
            node,
            topic_name,
            varnames);
    }

    virtual ~TopicOutputHandlerFactory() { }
};

void registerTopicOutputHandlerFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr _node)
{
    // be sure to use the BASE CLASS as template parameter for the Registry!
    Registry<OutputHandlerFactory>::registerFactory(std::make_shared<TopicOutputHandlerFactory>(_node));
}

} // namespace