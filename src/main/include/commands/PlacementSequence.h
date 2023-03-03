#pragma once

#include "subsystems/ArmSubsystem.h"
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class PlacementSequence : public frc2::CommandHelper<frc2::CommandBase, PlacementSequence>
{
    public:
    PlacementSequence(ArmSubsystem* arm);

    void Initialize() override;

    void Execute() override;

    void End(bool finished) override;

    bool IsFinished() override;

    private:
    ArmSubsystem* m_arm;
    double position;
    double startTime;
    bool dropFlag;
};