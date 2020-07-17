//
// Created by michael on 7/17/20.
//

#ifndef BR_HARDWARE_INTERFACE_PID_H
#define BR_HARDWARE_INTERFACE_PID_H

#include <chrono>
#include <cmath>
#include <algorithm>

class PID{
    double p_{0}, i_{0}, d_{0};
    double min_i_{0}, max_i_{0};
    bool use_anti_windup_{true};

    double last_error_{0};
    double integral_{0};

public:
    double update(double error, std::chrono::steady_clock::duration dt);

    void set_p(double p);
    void set_i(double i);
    void set_d(double d);
    void set_i_clamp_min(double min_i);
    void set_i_clamp_max(double max_i);
    void set_anti_windup(bool use_anti_windup);
};

#endif //BR_HARDWARE_INTERFACE_PID_H
