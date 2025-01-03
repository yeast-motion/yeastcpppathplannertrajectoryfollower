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
            void begin(Trajectory trajectory);

            MotionCommand follow(MotionState motion_state);
            FollowerStatus status();

        private:
            frc::Pose2d get_robot_pose();
            frc::ChassisSpeeds get_robot_speeds();
            bool get_should_flip();

            void yield_robot_output(const frc::ChassisSpeeds& speeds, const pathplanner::DriveFeedforwards& feedforwards);

            std::unique_ptr<pathplanner::FollowPathCommand> follow_path_command;
            std::shared_ptr<pathplanner::PathPlannerPath> path;
            std::shared_ptr<pathplanner::PathFollowingController> controller;
            nlohmann::json config_json;
    };
}