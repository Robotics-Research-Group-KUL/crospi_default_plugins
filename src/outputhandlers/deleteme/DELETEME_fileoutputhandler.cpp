#include "crospi_utils/fileoutputhandler.hpp"

#include "crospi_utils/string_interpolate.hpp"
#include "crospi_utils/etasl_error.hpp"
#include <fmt/format.h>

namespace etasl {

FileOutputHandler::FileOutputHandler(
    const std::string& filename,
    bool _header,
    size_t _maxlines,
    const std::vector<std::string>& _varnames)
    : out(string_interpolate(filename))
    , header(_header)
    , maxlines(_maxlines)
    , varnames(_varnames)
{
    name = fmt::format("FileOutputHandler({})", filename);
}

const std::string& FileOutputHandler::getName() const {
    return name;
}

bool FileOutputHandler::initialize(
    Context::Ptr ctx,
    const std::vector<std::string>& jnames,
    const std::vector<std::string>& fnames)
{
    if (varnames.size() == 0) {
        for (auto ov : ctx->output_vars) {
            auto ptr = ctx->getOutputExpression<double>(ov.first);
            if (ptr != 0) {
                Buffer buf;
                buf.outp = ptr;
                buf.name = cut_global(ov.first);
                lst.push_back(buf);
            }
        }
    } else {
        lst.reserve(varnames.size());
        for (auto i = 0; i < varnames.size(); ++i) {
            Buffer buf;
            buf.name = varnames[i];
            buf.outp = ctx->getOutputExpression<double>(buf.name);
            if (!buf.outp) {
                throw etasl_error(etasl_error::OUTPUTHANDLER_UKNOWN_OUTPUT, "fileoutputhandler to file '{}' can't find the scalar output expression '{}' (should be scalar!)", filename, buf.name);
                return false;
            }
            lst.push_back(buf);
        }
    }
    if (header) {
        for (auto buf : lst) {
            out << fmt::format("{}, ", buf.name);
        }
        out << std::endl;
    }
    counter = 0;
    maxcounter = lst.size() * maxlines;
    data.reserve(maxcounter); // maxlines is just an indication
    return true;
}

void FileOutputHandler::update(
    const std::vector<std::string>& jnames,
    const Eigen::VectorXd& jpos,
    const Eigen::VectorXd& jvel,
    const std::vector<std::string>& fnames,
    const Eigen::VectorXd& fvel,
    const Eigen::VectorXd& fpos)
{
    double value;
    for (auto buf : lst) {
        value = buf.outp->value();
        data.push_back(value);
        counter++;
    }
    
    
}

void FileOutputHandler::finalize()
{
    std::string line;
    size_t c = 0;
    while (c < counter) {
        line = "";
        for (size_t i = 0; i < lst.size(); ++i) {
            out << fmt::format("{}, ", data[c]);
            c++;
        }
        out << "\n";
    }
    out.close();
}

} // namespace etasl