#include "commands/Balance.h"
#include <frc/Timer.h>
Balance::Balance(Drivetrain* drive) : m_drive{drive}
{
    AddRequirements({drive});
    printf("Balance Constructor\n");
}

void Balance::Initialize()
{
    printf("Balance Init\n");
    maxPitch = 15;
    maxSpeed = 1.25;
    pitchTolerance = 0.5;
    balanceDuration = 2;
    timerStarted = false;
    debugTimestamp = frc::Timer::GetFPGATimestamp().value();
    levelAngle = 0;
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
    double localTimestamp = frc::Timer::GetFPGATimestamp().value();
    if(localTimestamp - debugTimestamp > 1)
    {
        printf("Execute %0.3f\n", localTimestamp);
        debugTimestamp = localTimestamp;
    }
    double roll = -m_drive->getRoll();
    double voltage = std::max(-maxSpeed, std::min(maxSpeed, maxSpeed * (roll - levelAngle) / maxPitch));
    printf("Volts %.03f Roll %0.3f\n", voltage, roll);
    m_drive->tankDriveVolts(units::volt_t{voltage}, units::volt_t{voltage});
}
bool Balance::IsFinished()
{
    
    double localTimestamp = frc::Timer::GetFPGATimestamp().value();
    bool printMsg = true;
    if(localTimestamp - debugTimestamp > 1)
    {
        debugTimestamp = localTimestamp;
        printMsg = true;
    }
    if(-m_drive->getRoll() > levelAngle-pitchTolerance && -m_drive->getRoll() < levelAngle + pitchTolerance)
    {
        if (!timerStarted)
        {
            if(printMsg)
                printf("IsFinished, Start Timer %0.3f\n", localTimestamp);
            timerStarted = true;
            timer = frc::Timer::GetFPGATimestamp().value();
            return false;
        }
        else if(frc::Timer::GetFPGATimestamp().value() - timer > balanceDuration)
        {
            if(printMsg)
                printf("IsFinished, Balance Achieved %0.3f\n", localTimestamp);
            return true;
        }
    }
    else if (timerStarted)
    {
        if(printMsg)
            printf("IsFinished, Timer Canceled %0.3f\n", localTimestamp);
        timerStarted = false;
    }
    return false;
}