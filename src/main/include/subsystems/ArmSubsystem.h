#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc2/command/ProfiledPIDSubsystem.h>
#include <frc/DigitalInput.h>

class ArmSubsystem : public frc2::SubsystemBase
{
    public:
    ArmSubsystem();

    void Init();

    void moveArm(units::volt_t volts);

    int intakeState = 0;

    void moveIntake(units::volt_t volts);

    void Periodic() override;

    double getEncoderValue();

    bool getLimitSwitch();


    private:
    rev::CANSparkMax m_armMotor{1, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_intakeMotor{2, rev::CANSparkMax::MotorType::kBrushed};
    rev::SparkMaxRelativeEncoder m_encoder{m_armMotor.GetEncoder()};
    frc::DigitalInput limitSwitch{0};
};

