#include "commands/IntakeSpeedCommand.h"

IntakeSpeedCommand::IntakeSpeedCommand(Intake & intake, double targetSpeed)
    : m_intake(intake), m_targetSpeed(targetSpeed)
{
    AddRequirements({&m_intake});
}

void IntakeSpeedCommand::Initialize()
{
    m_intake.SetIntakeSpeed(m_targetSpeed);
}

void IntakeSpeedCommand::End(bool interrupted)
{
    m_intake.SetIntakeSpeed(0);
}
