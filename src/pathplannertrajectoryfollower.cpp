#include <exception>
#include <functional>

#include "yeastcpppathplannertrajectoryfollower/pathplannertrajectoryfollower.hpp"
#include "pathplanner/lib/path/PathPlannerPath.h"
#include "pathplanner/lib/controllers/PPHolonomicDriveController.h"

#include "yeastcpppathplannertrajectoryfollower/extendedrobotconfig.hpp"

using namespace std;
using namespace yeast_motion;
using namespace pathplanner;

std::shared_ptr<PathPlannerPath> path_from_trajectory(Trajectory trajectory)
{
    std::vector<Waypoint> waypoints;
    for (size_t i = 0; i < trajectory.to_json()["waypoints"].size(); i++)
    {
        waypoints.push_back(Waypoint::fromJson(wpi::json::parse(trajectory.to_json()["waypoints"][i].dump())));
    }

    std::vector<RotationTarget> rotationTargets;
    for (size_t i = 0; i < trajectory.to_json()["rotationTargets"].size(); i++)
    {
        rotationTargets.push_back(RotationTarget::fromJson(wpi::json::parse(trajectory.to_json()["rotationTargets"][i].dump())));
    }

    std::vector<PointTowardsZone> pointTowardsZones;
    for (size_t i = 0; i < trajectory.to_json()["pointTowardsZones"].size(); i++)
    {
        pointTowardsZones.push_back(PointTowardsZone::fromJson(wpi::json::parse(trajectory.to_json()["pointTowardsZones"][i].dump())));
    }

    std::vector<ConstraintsZone> constraintZones;
    for (size_t i = 0; i < trajectory.to_json()["constraintZones"].size(); i++)
    {
        constraintZones.push_back(ConstraintsZone::fromJson(wpi::json::parse(trajectory.to_json()["constraintZones"][i].dump())));
    }

    std::vector<EventMarker> eventMarkers;
    for (size_t i = 0; i < trajectory.to_json()["eventMarkers"].size(); i++)
    {
        eventMarkers.push_back(EventMarker::fromJson(wpi::json::parse(trajectory.to_json()["eventMarkers"][i].dump())));
    }

    PathConstraints globalConstraints = PathConstraints::fromJson((wpi::json::parse((trajectory.to_json()["globalConstraints"]).dump())));

    IdealStartingState idealStartingState = IdealStartingState::fromJson((wpi::json::parse((trajectory.to_json()["idealStartingState"]).dump())));

    GoalEndState goalEndState = GoalEndState::fromJson((wpi::json::parse((trajectory.to_json()["goalEndState"]).dump())));

    bool reversed = (trajectory.to_json())["reversed"];

    std::shared_ptr<PathPlannerPath> path = std::make_shared<PathPlannerPath>
    (
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
        )
    );

	return path;
}

std::shared_ptr<PPHolonomicDriveController> controller_from_config(nlohmann::json json)
{
    nlohmann::json TranslationPIDjson = json["TranslationPIDConstants"];
    PIDConstants translationConstants(TranslationPIDjson["kP"], TranslationPIDjson["kI"], TranslationPIDjson["kD"], TranslationPIDjson["iZone"]);

    nlohmann::json RotationPIDjson = json["RotationPIDConstants"];
    PIDConstants rotationConstants(RotationPIDjson["kP"], RotationPIDjson["kI"], RotationPIDjson["kD"], RotationPIDjson["iZone"]);

    units::time::second_t period = units::time::second_t (json["Period"].get<double>());

    std::shared_ptr<PPHolonomicDriveController> controller = std::make_shared<PPHolonomicDriveController>
    (
        PPHolonomicDriveController
        (
            translationConstants,
            rotationConstants,
            period
        )
    );

    return controller;
}

RobotConfig config_from_json(nlohmann::json json)
{
    return ExtendedRobotConfig::from_json(json);
}

void PathPlannerTrajectoryFollower::set_config(nlohmann::json config)
{
    this->config_json = config;
}

void PathPlannerTrajectoryFollower::begin(Trajectory trajectory)
{
    frc2::Requirements requirements;

    path = path_from_trajectory(trajectory);
    controller = controller_from_config(config_json);
    std::cout << __LINE__ << std::endl;

    this->follow_path_command.release();
    this->follow_path_command.reset(nullptr);
    this->follow_path_command.reset
    (
        new FollowPathCommand
        (
            path,
			std::bind(&PathPlannerTrajectoryFollower::get_robot_pose, this),
			std::bind(&PathPlannerTrajectoryFollower::get_robot_speeds, this),
            std::bind(&PathPlannerTrajectoryFollower::yield_robot_output, this, placeholders::_1, placeholders::_2),
			controller,
			config_from_json(config_json),
            std::bind(&PathPlannerTrajectoryFollower::get_should_flip, this),
			requirements
        )
    );

    std::cout << __LINE__ << std::endl;

    this->follow_path_command->Initialize();
}

MotionCommand PathPlannerTrajectoryFollower::follow(MotionState motion_state)
{
    frc::Pose2d pose
       (units::length::meter_t(motion_state.measurement.pose.translation.x),
        units::length::meter_t(motion_state.measurement.pose.translation.y),
        frc::Rotation2d(units::radian_t(motion_state.measurement.pose.rotation.theta)));

    this->robot_pose = pose;

    frc::ChassisSpeeds speeds;
    speeds.vx = units::velocity::meters_per_second_t(motion_state.measurement.velocity.x);
    speeds.vy = units::velocity::meters_per_second_t(motion_state.measurement.velocity.y);
    speeds.omega = units::angular_velocity::radians_per_second_t(motion_state.measurement.velocity.omega);

    this->robot_chassis_speed = speeds;

    if (!this->follow_path_command->IsFinished())
    {
        this->follow_path_command->Execute();
    }

    MotionCommand output;
    output.velocity.x = this->command_speed.vx.value();
    output.velocity.y = this->command_speed.vy.value();
    output.velocity.omega = this->command_speed.omega.value();

    return output;
}

FollowerStatus PathPlannerTrajectoryFollower::status()
{
    for (auto& i : path->getEventMarkers())
    {
        if(i.getCommand()->IsFinished())
        {
            // std::cout << "Hit event: " << i.getTriggerName() << std::endl;
        }
    }

    return FollowerStatus(nlohmann::json());
}

void PathPlannerTrajectoryFollower::yield_robot_output(const frc::ChassisSpeeds& speeds, const pathplanner::DriveFeedforwards& feedforwards)
{
    command_speed = speeds;
    command_feed_forwards = feedforwards;
}

frc::Pose2d PathPlannerTrajectoryFollower::get_robot_pose()
{
    return robot_pose;
}

frc::ChassisSpeeds PathPlannerTrajectoryFollower::get_robot_speeds()
{
    return robot_chassis_speed;
}

bool PathPlannerTrajectoryFollower::get_should_flip()
{
    return false;
}