#include "commands/Balance.h"

Balance::Balance(Drivetrain* drive) : m_drive{drive}
{
    AddRequirements({drive});
}

void Balance::Execute()
{
    if(m_drive->getPitch() > 0)
    {
        m_drive->ArcadeDrive(.3, 0);
    }
    else if(m_drive->getPitch() < 0)
    {
        m_drive->ArcadeDrive(-.3, 0);
    }
    else m_drive->ArcadeDrive(0,0);
}