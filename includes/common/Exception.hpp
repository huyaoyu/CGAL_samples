//
// Created by yaoyu on 10/7/20.
//

#ifndef INCLUDES_COMMON_EXCEPTION_HPP
#define INCLUDES_COMMON_EXCEPTION_HPP


#include <exception>
#include <iostream>
#include <sstream>
#include <string>

#include <boost/exception/all.hpp>

namespace common {

struct ExceptionCommonBase : virtual std::exception, virtual boost::exception {};
struct NotImplemented : virtual ExceptionCommonBase {};
struct FileNotGood : virtual ExceptionCommonBase {};
struct FileExists : virtual ExceptionCommonBase {};

typedef boost::error_info<struct tag_info_string, std::string> ExceptionInfoString;

#define EXCEPTION_NOT_IMPLEMENTED(name) \
    {\
        std::stringstream name##_ss; \
        name##_ss << #name << " is not implemented. "; \
        BOOST_THROW_EXCEPTION( NotImplemented() << ExceptionInfoString(name##_ss.str()) ); \
    }

#define EXCEPTION_FILE_NOT_GOOD(fn) \
    {\
        std::stringstream fn##_ss; \
        fn##_ss << "File " << fn << " is not good. "; \
        BOOST_THROW_EXCEPTION( FileNotGood() << ExceptionInfoString(fn##_ss.str()) ); \
    }

#define EXCEPTION_FILE_EXISTS(fn) \
    {\
        std::stringstream fn##_ss; \
        fn##_ss << "File " << fn << " exists. "; \
        BOOST_THROW_EXCEPTION( FileExists() << ExceptionInfoString(fn##_ss.str()) ); \
    }

#define EXCEPTION_EIGEN_ROW_MAJOR(m) \
    {\
        std::stringstream m##_ss; \
        m##_ss << #m << " is row major. "; \
        BOOST_THROW_EXCEPTION( eigen_row_major() << ExceptionInfoString( m##_ss.str() ) ); \
    }

#define EXCEPTION_DIAG_INFO(ex) \
    boost::diagnostic_information(ex)

} // namespace common.

#endif //INCLUDES_COMMON_EXCEPTION_HPP
