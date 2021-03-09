#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Drivetrain.h>
#include <frc/Joystick.h>

#include "utility/Conditioning.h"

class TeleopDriveCommand : public frc2::CommandHelper<frc2::CommandBase, TeleopDriveCommand>
{
public:
    explicit TeleopDriveCommand(Drivetrain & drivetrain, frc::Joystick & driveJoystick, frc::Joystick& leftTankDriveJoystick);

    void Initialize() override;
    void Execute() override;

private:
    Drivetrain & m_drivetrain;
    frc::Joystick & m_driveJoystick;
    frc::Joystick & m_leftTankDriveJoystick;

    bool m_usingCurvatureDrive = true;

    cwtech::UniformConditioning m_conditioning{};
    cwtech::UniformConditioning m_turnCondiditioning{};
};
