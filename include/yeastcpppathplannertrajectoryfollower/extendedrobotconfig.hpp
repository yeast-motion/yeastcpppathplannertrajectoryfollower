#include <nlohmann/json.hpp>
#include "pathplanner/lib/config/RobotConfig.h"

namespace pathplanner
{
    class ExtendedRobotConfig : public RobotConfig
    {
        public:
            static RobotConfig from_json(nlohmann::json json);
            static frc::DCMotor getMotorFromSettingsString(std::string motorStr,
                    int numMotors);
    };
}