#include "subsystems/Drivetrain.h"
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

class Balance : public frc2::CommandHelper<frc2::CommandBase, Balance>
{
    public:
    Balance(Drivetrain* drive);
    void Execute() override;

    private:
    Drivetrain* m_drive;
    double pitch;
};