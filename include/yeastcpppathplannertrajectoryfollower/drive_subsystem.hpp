#pragma once

#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsControlModule.h>
#include <frc2/command/SubsystemBase.h>

class DriveSubsystem : public frc2::SubsystemBase {
    public:
        DriveSubsystem();

        void Periodic() override;

        void InitSendable(wpi::SendableBuilder& builder) override;
};