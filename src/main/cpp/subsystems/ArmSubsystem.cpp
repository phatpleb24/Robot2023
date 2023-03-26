#include "subsystems/ArmSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

ArmSubsystem::ArmSubsystem()
{
    Init();
}

void ArmSubsystem::Init()
{
    m_armMotor.SetInverted(true);
    //m_intakeMotor.SetSmartCurrentLimit(20, 20);
    m_armMotor.SetSmartCurrentLimit(60);
    frc::SmartDashboard::PutNumber("Hold Volts", armHoldVolts);
}

void ArmSubsystem::moveArm(units::volt_t volts)
{
    //if(limitSwitch.Get()) m_armMotor.SetVoltage(0_V);
    //else
    if(armStall)
        m_armMotor.SetVoltage(units::volt_t{-armHoldVolts});
    else 
        m_armMotor.SetVoltage(volts);
}

void ArmSubsystem::moveIntake(units::volt_t volts)
{
    m_intakeMotor.SetVoltage(volts);
}

void ArmSubsystem::Periodic()
{
    if(m_encoder.GetVelocity() < 0.01 && m_encoder.GetVelocity() > -0.01 && m_armMotor.GetOutputCurrent() > 10)
        armStall = 1;
    frc::SmartDashboard::PutNumber("Arm motor voltage", m_armMotor.GetAppliedOutput() * m_armMotor.GetBusVoltage());
    frc::SmartDashboard::PutNumber("Intake motor voltage", m_intakeMotor.GetMotorOutputVoltage() * m_intakeMotor.GetBusVoltage());
    frc::SmartDashboard::PutNumber("Arm motor temp", m_armMotor.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("Intake motor temp", m_intakeMotor.GetTemperature());
    frc::SmartDashboard::PutNumber("Intake State", intakeState);
    frc::SmartDashboard::PutNumber("Limit Switch", limitSwitch.Get());
    frc::SmartDashboard::PutNumber("Intake Current", m_intakeMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Arm Current", m_armMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Arm Encoder", m_encoder.GetPosition());
    armHoldVolts = frc::SmartDashboard::GetNumber("Hold Volts", armHoldVolts);
    frc::SmartDashboard::PutNumber("Arm velocity", m_encoder.GetVelocity());
    frc::SmartDashboard::PutBoolean("Arm Stall", armStall);
}

double ArmSubsystem::getEncoderValue()
{
    return m_encoder.GetPosition();
}

bool ArmSubsystem::getLimitSwitch()
{
    return limitSwitch.Get();
}