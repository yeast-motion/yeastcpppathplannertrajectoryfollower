#include "yeastcpppathplannertrajectoryfollower/bogus_subsystem.hpp"

#include <wpi/sendable/SendableBuilder.h>
#include <iostream>

BogusSubsystem::BogusSubsystem() {}

void BogusSubsystem::InitSendable(wpi::SendableBuilder& builder) {
    SubsystemBase::InitSendable(builder);
}

void BogusSubsystem::Periodic()
{
    std::cout << "I am running the Bogus subsystem!" << std::endl;
}