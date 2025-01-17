#pragma once

#include <iostream>
#include <vector>
#include <memory>


#include "yeastcpp/components/follower.hpp"

#include "pathplanner/lib/commands/FollowPathCommand.h"
#include "pathplanner/lib/util/DriveFeedforwards.h"
#include "pathplanner/lib/path/PathPlannerPath.h"
#include "pathplanner/lib/controllers/PathFollowingController.h"

namespace yeast_motion
{
    class PathPlannerTrajectoryFollower : public Follower
    {
        public:
            void set_config(nlohmann::json config);

            void begin(std::shared_ptr<pathplanner::PathPlannerPath> path, MotionState initial_state = MotionState());
            void begin(Trajectory trajectory, MotionState initial_state = MotionState());
            void begin_choreo(std::string file_path, std::string trajectory_name, MotionState initial_state = MotionState());
            void begin_choreo(std::string file_path, std::string trajectory_name, size_t split_index, MotionState initial_state = MotionState());

            MotionCommand follow(MotionState motion_state);
            FollowerStatus status();

            static std::vector<yeast_motion::Pose2D> get_path_poses(std::shared_ptr<pathplanner::PathPlannerPath> path);
            static std::shared_ptr<pathplanner::PathPlannerPath> path_from_trajectory(Trajectory trajectory);
            static std::shared_ptr<pathplanner::PathPlannerPath> path_from_choreo(std::string file_path, std::string trajectory_name, size_t split_index=0);

        private:
            void set_motion_state(MotionState state);
            frc::Pose2d get_robot_pose();
            frc::ChassisSpeeds get_robot_speeds();
            bool get_should_flip();
            void register_named_commands(nlohmann::json event_markers);
            void register_named_commands(std::vector<pathplanner::EventMarker> event_markers);
            void register_named_command(std::string name);

            void log_command(std::string name);

            void yield_robot_output(const frc::ChassisSpeeds& speeds, const pathplanner::DriveFeedforwards& feedforwards);

            std::unique_ptr<pathplanner::FollowPathCommand> follow_path_command;
            std::shared_ptr<pathplanner::PathPlannerPath> path;
            std::shared_ptr<pathplanner::PathFollowingController> controller;

            frc::ChassisSpeeds command_speed;
            pathplanner::DriveFeedforwards command_feed_forwards;

            frc::Pose2d robot_pose;
            frc::ChassisSpeeds robot_chassis_speed;

            std::vector<std::string> passed_commands;

            nlohmann::json config_json;
    };
}