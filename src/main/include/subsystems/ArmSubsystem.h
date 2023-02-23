#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc2/command/ProfiledPIDSubsystem.h>

class ArmSubsystem : public frc2::SubsystemBase
{
    public:
    ArmSubsystem();

    void Init();

    void moveArm(units::volt_t volts);

    void moveIntake(units::volt_t volts);


    private:
    rev::CANSparkMax m_armMotor{1, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_intakeMotor{2, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkMaxRelativeEncoder m_encoder{m_armMotor.GetEncoder()};
};