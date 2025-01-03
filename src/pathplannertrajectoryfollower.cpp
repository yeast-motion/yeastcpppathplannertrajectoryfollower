#include <exception>

#include "yeastcpppathplannertrajectoryfollower/pathplannertrajectoryfollower.hpp"

#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/SwerveDriveOdometry.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Pose2d.h"

using namespace yeast_motion;


void PathPlannerTrajectoryFollower::begin(Trajectory trajectory)
{

}

MotionCommand PathPlannerTrajectoryFollower::follow(MotionState motion_state)
{
    return MotionCommand();
}

FollowerStatus PathPlannerTrajectoryFollower::status()
{
    return FollowerStatus(nlohmann::json());
}