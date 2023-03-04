#include "subsystems/ArmSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

ArmSubsystem::ArmSubsystem()
{
    Init();
}

void ArmSubsystem::Init()
{
    m_armMotor.SetInverted(true);
    m_intakeMotor.SetSmartCurrentLimit(25);
    m_armMotor.SetSmartCurrentLimit(60);
}

void ArmSubsystem::moveArm(units::volt_t volts)
{
    //if(limitSwitch.Get()) m_armMotor.SetVoltage(0_V);
    //else
     m_armMotor.SetVoltage(volts);
}

void ArmSubsystem::moveIntake(units::volt_t volts)
{
    m_intakeMotor.SetVoltage(volts);
}

void ArmSubsystem::Periodic()
{
    frc::SmartDashboard::PutNumber("Arm motor voltage", m_armMotor.GetAppliedOutput() * m_armMotor.GetBusVoltage());
    frc::SmartDashboard::PutNumber("Intake motor voltage", m_intakeMotor.GetAppliedOutput() * m_intakeMotor.GetBusVoltage());
    frc::SmartDashboard::PutNumber("Arm motor temp", m_armMotor.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("Intake motor temp", m_intakeMotor.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("Limit Switch", limitSwitch.Get());
    frc::SmartDashboard::PutNumber("Intake Current", m_intakeMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Arm Current", m_armMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Arm Encoder", m_encoder.GetPosition());
}

double ArmSubsystem::getEncoderValue()
{
    return m_encoder.GetPosition();
}

bool ArmSubsystem::getLimitSwitch()
{
    return limitSwitch.Get();
}