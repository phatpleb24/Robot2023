#include "commands/Balance.h"
#include <frc/Timer.h>
Balance::Balance(Drivetrain* drive) : m_drive{drive}
{
    AddRequirements({drive});
}

void Balance::Initialize()
{
    maxPitch = 15;
    maxSpeed = 0.3;
    pitchTolerance = 0.5;
    balanceDuration = 1;
}

void Balance::Execute()
{
    /*if(m_drive->getPitch() > 3)
    {
        m_drive->ArcadeDrive(.3, 0);
    }
    else if(m_drive->getPitch() < -3)
    {
        m_drive->ArcadeDrive(-.3, 0);
    }
    else m_drive->ArcadeDrive(0,0);*/
    m_drive->ArcadeDrive(maxSpeed * m_drive->getPitch() / maxPitch, 0);
}
bool Balance::IsFinished()
{
    if(m_drive->getPitch() > -pitchTolerance && m_drive->getPitch() < pitchTolerance)
    {
        if (!timerStarted)
        {
            timerStarted = true;
            timer = frc::Timer::GetFPGATimestamp().value();
            return false;
        }
        else if(frc::Timer::GetFPGATimestamp().value() - timer > balanceDuration)
        {
            return true;
        }
    }
    else if (timerStarted)
    {
        timerStarted = false;
    }
    return false;
}