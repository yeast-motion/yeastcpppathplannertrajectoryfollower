#include <exception>
#include <functional>

#include "yeastcpppathplannertrajectoryfollower/pathplannertrajectoryfollower.hpp"
#include "pathplanner/lib/path/PathPlannerPath.h"
#include "pathplanner/lib/controllers/PPHolonomicDriveController.h"
#include "frc2/command/Command.h"
#include "pathplanner/lib/auto/NamedCommands.h"

#include "yeastcpppathplannertrajectoryfollower/extendedrobotconfig.hpp"

using namespace std;
using namespace yeast_motion;
using namespace pathplanner;

void PathPlannerTrajectoryFollower::print_the_guy(std::string name)
{
    std::cout << "Event: " << name << std::endl;
}

void PathPlannerTrajectoryFollower::register_named_command(std::string name)
{
    std::cout << "Registering command: " << name << std::endl;
    // EventTrigger(name).OnTrue(frc2::cmd::RunOnce([this, name] { this->print_the_guy(name); }));
    // NamedCommands::registerCommand(name,frc2::cmd::Print("Yeouch"));// frc2::cmd::RunOnce([this, name] { this->print_the_guy(name); }));
    auto test = frc2::cmd::RunOnce([this, name] { this->print_the_guy(name); }, { &bogus_subsystem });
    std::cout << "Todd Test: " << std::endl;
    auto bad = test.get()->GetRequirements();
    std::cout << "Req Size: " << bad.size() << std::endl;
    for (auto i : test.get()->GetRequirements())
    {
        std::cout << "Here have: " << i->GetName() << std::endl;
    }
    std::cout << "Iterated requirements" << std::endl;
    std::shared_ptr<frc2::Command> command = std::make_shared<frc2::Command> (test.get());

    NamedCommands::registerCommand(name, command); //frc2::cmd::RunOnce([this, name] { this->print_the_guy(name); }, { &bogus_subsystem }));
    auto mycommandafter = NamedCommands::getCommand(name);
    std::cout << "My command after size: " << mycommandafter.get()->GetRequirements().size() << std::endl;

    for (auto i : mycommandafter.get()->GetRequirements())
    {
        std::cout << "After Here have: " << i->GetName() << std::endl;
    }
    std::cout << "Command registered" << std::endl;
}

void PathPlannerTrajectoryFollower::register_named_commands(nlohmann::json event_markers)
{
    register_named_command("todd_command");

    // for (auto& marker : event_markers)
    // {
    //     std::cout << "Registering event: " << marker["name"] << std::endl;
    //     register_named_command(marker["name"]);
    // }
}

std::shared_ptr<PathPlannerPath> PathPlannerTrajectoryFollower::path_from_trajectory(Trajectory trajectory)
{
    std::cout << "Going to place: " << std::endl;
    register_named_commands(trajectory.to_json()["eventMarkers"]);

    std::cout << "Going to start allocating nonsense: " << std::endl;

    std::vector<Waypoint> waypoints;
    for (size_t i = 0; i < trajectory.to_json()["waypoints"].size(); i++)
    {
        waypoints.push_back(Waypoint::fromJson(wpi::json::parse(trajectory.to_json()["waypoints"][i].dump())));
    }

    std::cout << __FILE__ << ":" << __LINE__ << std::endl;

    std::vector<RotationTarget> rotationTargets;
    for (size_t i = 0; i < trajectory.to_json()["rotationTargets"].size(); i++)
    {
        rotationTargets.push_back(RotationTarget::fromJson(wpi::json::parse(trajectory.to_json()["rotationTargets"][i].dump())));
    }

    std::cout << __FILE__ << ":" << __LINE__ << std::endl;

    std::vector<PointTowardsZone> pointTowardsZones;
    for (size_t i = 0; i < trajectory.to_json()["pointTowardsZones"].size(); i++)
    {
        pointTowardsZones.push_back(PointTowardsZone::fromJson(wpi::json::parse(trajectory.to_json()["pointTowardsZones"][i].dump())));
    }

    std::cout << __FILE__ << ":" << __LINE__ << std::endl;

    std::vector<ConstraintsZone> constraintZones;
    for (size_t i = 0; i < trajectory.to_json()["constraintZones"].size(); i++)
    {
        constraintZones.push_back(ConstraintsZone::fromJson(wpi::json::parse(trajectory.to_json()["constraintZones"][i].dump())));
    }

    std::cout << __FILE__ << ":" << __LINE__ << std::endl;

    std::vector<EventMarker> eventMarkers;
    for (size_t i = 0; i < trajectory.to_json()["eventMarkers"].size(); i++)
    {
        eventMarkers.push_back(EventMarker::fromJson(wpi::json::parse(trajectory.to_json()["eventMarkers"][i].dump())));
    }

    std::cout << __FILE__ << ":" << __LINE__ << std::endl;

    PathConstraints globalConstraints = PathConstraints::fromJson((wpi::json::parse((trajectory.to_json()["globalConstraints"]).dump())));

    std::cout << __FILE__ << ":" << __LINE__ << std::endl;
    IdealStartingState idealStartingState = IdealStartingState::fromJson((wpi::json::parse((trajectory.to_json()["idealStartingState"]).dump())));

    std::cout << __FILE__ << ":" << __LINE__ << std::endl;
    GoalEndState goalEndState = GoalEndState::fromJson((wpi::json::parse((trajectory.to_json()["goalEndState"]).dump())));

    std::cout << __FILE__ << ":" << __LINE__ << std::endl;
    bool reversed = (trajectory.to_json())["reversed"];

    std::cout << __FILE__ << ":" << __LINE__ << std::endl;
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
    std::cout << "Created path" << std::endl;

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
    frc2::CommandScheduler::GetInstance().Run();
    frc2::Requirements requirements;

    path = path_from_trajectory(trajectory);
    controller = controller_from_config(config_json);

    std::cout << "Controller created" << std::endl;



    std::cout << "Going to make command" << std::endl;

    // this->follow_path_command.release();
    // this->follow_path_command.reset(nullptr);
    // this->follow_path_command.reset
    // (
    //     new FollowPathCommand
    //     (
    //         path,
	// 		std::bind(&PathPlannerTrajectoryFollower::get_robot_pose, this),
	// 		std::bind(&PathPlannerTrajectoryFollower::get_robot_speeds, this),
    //         std::bind(&PathPlannerTrajectoryFollower::yield_robot_output, this, placeholders::_1, placeholders::_2),
	// 		controller,
	// 		config_from_json(config_json),
    //         std::bind(&PathPlannerTrajectoryFollower::get_should_flip, this),
	// 		{&drive_subsystem}
    //     )
    // );

    // frc2::CommandScheduler::GetInstance().Schedule({ this->follow_path_command.get() });

    frc2::CommandScheduler::GetInstance().Schedule({
        new FollowPathCommand
        (
            path,
			std::bind(&PathPlannerTrajectoryFollower::get_robot_pose, this),
			std::bind(&PathPlannerTrajectoryFollower::get_robot_speeds, this),
            std::bind(&PathPlannerTrajectoryFollower::yield_robot_output, this, placeholders::_1, placeholders::_2),
			controller,
			config_from_json(config_json),
            std::bind(&PathPlannerTrajectoryFollower::get_should_flip, this),
			{&drive_subsystem}
        )
    });


    // this->follow_path_command->Initialize();
}

MotionCommand PathPlannerTrajectoryFollower::follow(MotionState motion_state)
{
    frc2::CommandScheduler::GetInstance().Run();
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
        std::cout << "Events: " << i.getTriggerName() << std::endl;
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