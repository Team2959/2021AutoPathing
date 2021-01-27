#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include "RobotMap.h"

class Intake : public frc2::SubsystemBase
{
private:
    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_intake {kIntakeVictorSpxCanId};

public:
    Intake();

    void SetSpeed(double speed);
};
