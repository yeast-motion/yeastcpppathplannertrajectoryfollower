#include <exception>
#include <functional>

#include "yeastcpppathplannertrajectoryfollower/pathplannertrajectoryfollower.hpp"
#include "pathplanner/lib/path/PathPlannerPath.h"

using namespace std;
using namespace yeast_motion;
using namespace pathplanner;

void PathPlannerTrajectoryFollower::begin(Trajectory trajectory)
{   
    RobotConfig * config;
    frc2::Requirements requirements;

    this->follow_path_command.reset
    (
        new FollowPathCommand
        (
            path,
			std::bind(&PathPlannerTrajectoryFollower::get_robot_pose, this),
			std::bind(&PathPlannerTrajectoryFollower::get_robot_speeds, this),
            std::bind(&PathPlannerTrajectoryFollower::yield_robot_output, this, placeholders::_1, placeholders::_2),
			controller,
			*config,
            std::bind(&PathPlannerTrajectoryFollower::get_should_flip, this),
			requirements
        )
    );
}

MotionCommand PathPlannerTrajectoryFollower::follow(MotionState motion_state)
{
    return MotionCommand();
}

FollowerStatus PathPlannerTrajectoryFollower::status()
{
    return FollowerStatus(nlohmann::json());
}