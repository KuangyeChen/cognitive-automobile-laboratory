//
// Created by kal3-1 on 01.06.18.
//

#pragma once

namespace kitaf_utils_ros {

    enum DetectionSignal {
        SEE_NO_TRAFFIC_SIGN = 0,
        MIN_DEFINE = SEE_NO_TRAFFIC_SIGN,
        LEFT_SIGN = 1,
        RIGHT_SIGN = 2,
        FORWARD_SIGN = 3,
        STOP_SIGN = 4,
        MAX_DEFINE = STOP_SIGN
    };

} // namespace kitaf_utils