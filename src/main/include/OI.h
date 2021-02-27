#pragma once

#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>
#include "RobotMap.h"

class OI
{
public:
    OI();

    frc::Joystick m_driverJoystick{0};
    frc::Joystick m_buttonBox{1};

    frc2::JoystickButton m_reverseKickerButton{&m_buttonBox, kReverseKicker};
    frc2::JoystickButton m_reverseConveyorButton{&m_buttonBox, kReverseConveyor};
    frc2::JoystickButton m_reverseIntakeButton{&m_buttonBox, kReverseIntake};
    frc2::JoystickButton m_intakeToggleButton{&m_buttonBox, kIntakeToggle};
};
