#include "subsystems/Drivetrain.h"
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

class Balance : public frc2::CommandHelper<frc2::CommandBase, Balance>
{
    public:
    Balance(Drivetrain* drive);
    void Execute() override;
    bool IsFinished() override;

    private:
    Drivetrain* m_drive;
    bool timerStarted;
    double pitch;
    double timer;
};