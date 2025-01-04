#include "yeastcpppathplannertrajectoryfollower/drive_subsystem.hpp"

#include <wpi/sendable/SendableBuilder.h>
#include <iostream>

DriveSubsystem::DriveSubsystem() {}

void DriveSubsystem::InitSendable(wpi::SendableBuilder& builder) {
    SubsystemBase::InitSendable(builder);
}

void DriveSubsystem::Periodic()
{
    std::cout << "I am running the drive subsystem!" << std::endl;
}