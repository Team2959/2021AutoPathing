#include "commands/TeleopDriveCommand.h"
#include "Robot.h"

TeleopDriveCommand::TeleopDriveCommand(Drivetrain* drivetrain, OI* oi)
 : m_drivetrain(drivetrain), m_oi(oi)
{
    AddRequirements({m_drivetrain});
}

void TeleopDriveCommand::Execute() 
{
    m_drivetrain->CurvatureDrive(
        m_oi->m_conditioning.Condition(-m_oi->m_driverJoystick.GetY()),
        m_oi->m_conditioning.Condition(m_oi->m_driverJoystick.GetTwist()),
        m_oi->m_quickTurn.Get());
}
