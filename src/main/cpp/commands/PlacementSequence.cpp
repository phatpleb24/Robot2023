#include "commands/PlacementSequence.h"
#include <frc/Timer.h>

PlacementSequence::PlacementSequence(ArmSubsystem* arm) : m_arm{arm}
{
    printf("Placement Constructor\n");
    AddRequirements({arm});
}

void PlacementSequence::Initialize()
{
    printf("Placement Init\n");
    position = m_arm->getEncoderValue();
    startTime = - 1;
    dropFlag = false;
}

void PlacementSequence::Execute()
{
    units::volt_t armVolt = 5_V;
    units::volt_t intakeVolt = 0_V;
        if(startTime == -1)
        {
            startTime = frc::Timer::GetFPGATimestamp().value();
        }
        if(frc::Timer::GetFPGATimestamp().value() - startTime >= 2.5)
        {
            intakeVolt = -9_V;
            armVolt = -5_V;
        }
        else 
            intakeVolt = 0_V;  
    printf("Execute\n");
    printf("Arm %.03f Intake %.03f", armVolt.value(), intakeVolt.value());
    m_arm->moveIntake(intakeVolt);
    m_arm->moveArm(armVolt);
}

bool PlacementSequence::IsFinished()
{
    return m_arm->getEncoderValue() <= position - 0.1;
}

void PlacementSequence::End(bool interrupted)
{
    printf("Placement End\n");
    m_arm->moveArm(0_V);
    m_arm->moveIntake(0_V);
}