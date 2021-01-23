#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Drivetrain.h>
#include "OI.h"

class TeleopDriveCommand : public frc2::CommandHelper<frc2::CommandBase, TeleopDriveCommand>
{
public:
    explicit TeleopDriveCommand(Drivetrain* drivetrain, OI* oi);
    void Execute() override;
private:
    Drivetrain* m_drivetrain;
    OI * m_oi;
};
