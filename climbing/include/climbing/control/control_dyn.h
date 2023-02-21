#ifndef CONTROL_DYN_GAURD_HPP
#define CONTROL_DYN_GAURD_HPP

#include<cmath>
#include<iostream>

namespace climbing
{
    constexpr uint32_t deg2tics(double deg){
        return static_cast<uint32_t>(deg*(4095/360));
    }

    double tics2deg(uint32_t tics){
        return static_cast<double>(tics)*(360.0/4095.0);
    }
}

#endif