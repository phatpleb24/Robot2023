#include "commands/PlacementSequence.h"
#include <frc/Timer.h>

PlacementSequence::PlacementSequence(ArmSubsystem* arm) : m_arm{arm}
{
    AddRequirements({arm});
}

void PlacementSequence::Initialize()
{
    position = m_arm->getEncoderValue();
    startTime = - 1;
    dropFlag = false;
}

void PlacementSequence::Execute()
{
    m_arm->moveArm(2_V);
    if(m_arm->getLimitSwitch())
    {
        if(startTime == -1)
        {
            startTime = frc::Timer::GetFPGATimestamp().value();
        }
        if(frc::Timer::GetFPGATimestamp().value() - startTime >= 2)
        {
            m_arm->moveIntake(0_V);
            m_arm->moveArm(-2_V);
        }
        else 
            m_arm->moveIntake(-2_V);   
    }
}

bool PlacementSequence::IsFinished()
{
    return m_arm->getEncoderValue() <= position && dropFlag;
}

void PlacementSequence::End(bool interrupted)
{
    m_arm->moveArm(0_V);
}