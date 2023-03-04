#include "commands/Balance.h"
#include <frc/Timer.h>
Balance::Balance(Drivetrain* drive) : m_drive{drive}
{
    AddRequirements({drive});
}

void Balance::Execute()
{
    if(m_drive->getPitch() > 3)
    {
        m_drive->ArcadeDrive(.3, 0);
    }
    else if(m_drive->getPitch() < -3)
    {
        m_drive->ArcadeDrive(-.3, 0);
    }
    else m_drive->ArcadeDrive(0,0);
}
bool Balance::IsFinished()
{
    if(m_drive->getPitch() > -3.0 && m_drive->getPitch() < 3.0)
    {
        if (!timerStarted)
        {
            timerStarted = true;
            timer = frc::Timer::GetFPGATimestamp().value();
            return false;
        }
        else if(frc::Timer::GetFPGATimestamp().value() - timer > 2.5)
        {
            return true;
        }
    }
    else if (timerStarted)
    {
        timer = frc::Timer::GetFPGATimestamp().value();
    }
    return false;
}