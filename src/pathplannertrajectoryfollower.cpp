#include <exception>
#include <functional>

#include "yeastcpppathplannertrajectoryfollower/pathplannertrajectoryfollower.hpp"
#include "pathplanner/lib/path/PathPlannerPath.h"
#include "pathplanner/lib/controllers/PPHolonomicDriveController.h"

using namespace std;
using namespace yeast_motion;
using namespace pathplanner;

std::shared_ptr<PathPlannerPath> path_from_trajectory(Trajectory trajectory)
{
    std::vector<Waypoint> waypoints;
    for (nlohmann::json i : (trajectory.to_json())["Waypoints"])
    {
        waypoints.push_back(Waypoint::fromJson(wpi::json(i.dump())));
    }

    std::vector<RotationTarget> rotationTargets;
    for (nlohmann::json i : (trajectory.to_json())["RotationTargets"])
    {
        rotationTargets.push_back(RotationTarget::fromJson(wpi::json(i.dump())));
    }

    std::vector<PointTowardsZone> pointTowardsZones;
    for (nlohmann::json i : (trajectory.to_json())["PointTowardsZones"])
    {
        pointTowardsZones.push_back(PointTowardsZone::fromJson(wpi::json(i.dump())));
    }

    std::vector<ConstraintsZone> constraintZones;
    for (nlohmann::json i : (trajectory.to_json())["ConstraintZones"])
    {
        constraintZones.push_back(ConstraintsZone::fromJson(wpi::json(i.dump())));
    }

    std::vector<EventMarker> eventMarkers;
    for (nlohmann::json i : (trajectory.to_json())["EventMarkers"])
    {
        eventMarkers.push_back(EventMarker::fromJson(wpi::json(i.dump())));
    }

    PathConstraints globalConstraints = PathConstraints::fromJson((wpi::json((trajectory.to_json()["GlobalConstraints"]).dump())));

    IdealStartingState idealStartingState = IdealStartingState::fromJson((wpi::json((trajectory.to_json()["IdealStartingState"]).dump())));

    GoalEndState goalEndState = GoalEndState::fromJson((wpi::json((trajectory.to_json()["GoalEndState"]).dump())));

    bool reversed = (trajectory.to_json())["Reversed"];

	PathPlannerPath
    (
        waypoints,
        rotationTargets,
        pointTowardsZones,
        constraintZones,
        eventMarkers,
        globalConstraints,
        idealStartingState,
        goalEndState, 
        reversed
    );
}

PPHolonomicDriveController * controller_from_config(nlohmann::json json)
{

}

void PathPlannerTrajectoryFollower::begin(Trajectory trajectory)
{   
    RobotConfig * config;
    frc2::Requirements requirements;

    path = path_from_trajectory(trajectory);
    controller.reset(controller_from_config(config_json));

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