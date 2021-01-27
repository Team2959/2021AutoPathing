#include "commands/IntakeSpeedCommand.h"

IntakeSpeedCommand::IntakeSpeedCommand(Intake & intake, double targetSpeed)
    : m_intake(intake), m_targetSpeed(targetSpeed)
{
    AddRequirements({&m_intake});
}

void IntakeSpeedCommand::Initialize()
{
    m_intake.SetSpeed(m_targetSpeed);
}

void IntakeSpeedCommand::End(bool interrupted)
{
    m_intake.SetSpeed(0);
}
