#pragma once

#include <frc/Joystick.h>
#include <frc/buttons/JoystickButton.h>
#include "utility/Conditioning.h"
#include "RobotMap.h"

class OI
{
public:
    OI();

    frc::Joystick m_driverJoystick{0};
    frc::JoystickButton m_quickTurn {&m_driverJoystick, kQuickTurn};
    cwtech::UniformConditioning m_conditioning{};
};
