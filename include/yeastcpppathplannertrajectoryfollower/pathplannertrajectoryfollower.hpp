#pragma once

#include <iostream>
#include <vector>


#include "yeastcpp/components/follower.hpp"


namespace yeast_motion
{
    class PathPlannerTrajectoryFollower : public Follower
    {
        public:
            void begin(Trajectory trajectory) = 0;

            MotionCommand follow(MotionState motion_state) = 0;
            FollowerStatus status() = 0;

        private:
    };
}