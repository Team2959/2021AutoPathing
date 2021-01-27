#include "subsystems/Intake.h"

Intake::Intake()
{
}

void Intake::SetSpeed(double speed)
{
    m_intake.Set(speed);
}
