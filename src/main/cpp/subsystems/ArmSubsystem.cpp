#include "subsystems/ArmSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

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
    if(volts > 0_V)
    {
        if(limitSwitch.Get())
        {
            m_intakeMotor.SetVoltage(0_V);
        }
        else m_intakeMotor.SetVoltage(volts);
    }
}

void ArmSubsystem::Periodic()
{
    frc::SmartDashboard::PutNumber("Arm motor voltage", m_armMotor.GetAppliedOutput() * m_armMotor.GetBusVoltage());
    frc::SmartDashboard::PutNumber("Intake motor voltage", m_intakeMotor.GetAppliedOutput() * m_intakeMotor.GetBusVoltage());
    frc::SmartDashboard::PutNumber("Arm motor temp", m_armMotor.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("Intake motor temp", m_intakeMotor.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("Limit Switch", limitSwitch.Get());
}

double ArmSubsystem::getEncoderValue()
{
    return m_encoder.GetPosition();
}

bool ArmSubsystem::getLimitSwitch()
{
    return limitSwitch.Get();
}