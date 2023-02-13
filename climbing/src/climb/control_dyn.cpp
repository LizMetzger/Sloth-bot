#include "control/control_dyn.h"

namespace climbing{
    constexpr int deg2tics(double deg){
        return deg*(4095/360);
    }

    constexpr double tics2deg(int tics){
        return tics*(360/4095);
    }
}