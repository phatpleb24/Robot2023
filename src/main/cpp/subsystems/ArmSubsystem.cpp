#include "subsystems/ArmSubsystem.h"

ArmSubsystem::ArmSubsystem()
{
    Init();
}

void ArmSubsystem::Init()
{
    m_armMotor.SetInverted(true);
}

void ArmSubsystem::moveArm(units::volt_t volts)
{
    m_armMotor.SetVoltage(volts);
}

void ArmSubsystem::moveIntake(units::volt_t volts)
{
    m_intakeMotor.SetVoltage(volts);
}