//
// Created by yaoyu on 10/20/20.
//

#ifndef INCLUDES_COMMON_SCOPEDTIMER_HPP
#define INCLUDES_COMMON_SCOPEDTIMER_HPP

#include <chrono>
#include <iostream>

#define FUNCTION_SCOPE_TIMER \
    common::ScopeTimer __func__##_Timer(__func__);

#define SCOPE_TIMER_NAME1(x, y) x##y
#define SCOPE_TIMER_NAME(x, y) SCOPE_TIMER_NAME1(x, y)
#define SCOPE_TIMER_NAME2(x) #x
#define SCOPE_TIMER_VARIABLE_NAME(x) SCOPE_TIMER_NAME2(x)
#define NAMED_SCOPE_TIMER(name) \
    common::ScopeTimer SCOPE_TIMER_NAME(name, __LINE__)( \
        SCOPE_TIMER_VARIABLE_NAME( SCOPE_TIMER_NAME(name, __LINE__) ) );

namespace common
{

class ScopeTimer
{
    std::chrono::high_resolution_clock::time_point t0;
    std::string name;
public:
    ScopeTimer(const std::string &name) : name(name) { t0 = std::chrono::high_resolution_clock::now(); }
    ~ScopeTimer() {
        double timestamp = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t0).count();
        std::cout << name << " elapses "<< timestamp << "s. \n";
    }
};

} // namespace comon


#endif //INCLUDES_COMMON_SCOPEDTIMER_HPP
