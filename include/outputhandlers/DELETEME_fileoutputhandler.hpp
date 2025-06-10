#pragma once

#include "etasl_task_utils/outputhandler.hpp"

#include <fstream>

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
class FileOutputHandler : public OutputHandler {
    struct Buffer {
        Expression<double>::Ptr outp;
        std::string name;
    };

    std::vector<Buffer> lst;
    std::vector<std::string> varnames;
    std::string filename;
    bool header;
    std::ofstream out;
    size_t linenr;
    size_t maxlines;
    size_t maxcounter;
    size_t counter;
    std::vector<double> data;
    std::string name;

public:
    /**
     * @brief
     * @param filename
     * @param header
     * @param _varnames
     */
    FileOutputHandler();

    /**
     * @return a name for the instance of this object 
     */
    const std::string& getName() const override;

    virtual bool construct(
        std::string name, 
        rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
        const Json::Value& _parameters,
        boost::shared_ptr<etasl::JsonChecker> _jsonchecker) override;

    /**
     * @brief
     * @param ctx
     * @param jnames
     * @param fnames
     */
    virtual bool initialize(
        Context::Ptr ctx,
        const std::vector<std::string>& jnames,
        const std::vector<std::string>& fnames) override;
    /**
     * @brief
     * @param jnames
     * @param jpos
     * @param jvel
     * @param fnames
     * @param fvel
     * @param fpos
     */
    virtual void update(
        const std::vector<std::string>& jnames,
        const Eigen::VectorXd& jpos, const Eigen::VectorXd& jvel,
        const std::vector<std::string>& fnames,
        const Eigen::VectorXd& fvel,
        const Eigen::VectorXd& fpos) override;

    /**
     * @brief
     */
    virtual void finalize();

}; // TopicOutputHandler
} // namespace KDL
