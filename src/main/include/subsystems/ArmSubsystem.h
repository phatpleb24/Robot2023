#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include <frc2/command/ProfiledPIDSubsystem.h>
#include <frc/DigitalInput.h>

class ArmSubsystem : public frc2::SubsystemBase
{
    public:
    ArmSubsystem();

    void Init();

    void moveArm(units::volt_t volts);

    int intakeState = 0;

    int armBack = 0;

    void moveIntake(units::volt_t volts);

    void Periodic() override;

    double getEncoderValue();

    bool getLimitSwitch();

    bool armStall = 0;
    
    private:
    rev::CANSparkMax m_armMotor{1, rev::CANSparkMax::MotorType::kBrushless};
    WPI_TalonFX m_intakeMotor{2};
    rev::SparkMaxRelativeEncoder m_encoder{m_armMotor.GetEncoder()};
    frc::DigitalInput limitSwitch{0};

    int stallCNT = 0;

    double armHoldVolts = 0.5;
};

