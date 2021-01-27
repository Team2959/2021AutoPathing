#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Intake.h>

class IntakeSpeedCommand : public frc2::CommandHelper<frc2::CommandBase, IntakeSpeedCommand>
{
public:
    explicit IntakeSpeedCommand(Intake & intake, double targetSpeed);

    void Initialize() override;
    void End(bool interrupted) override;

private:
    Intake & m_intake;
    double m_targetSpeed;
};
