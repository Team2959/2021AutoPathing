#pragma once

#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>
#include "RobotMap.h"

class OI
{
public:
    OI();

    frc::Joystick m_driverJoystick{0};

    frc2::JoystickButton m_intakeSpeedButton {&m_driverJoystick, kFire};
};
