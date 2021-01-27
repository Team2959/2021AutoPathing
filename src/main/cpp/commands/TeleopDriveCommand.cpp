#include "commands/TeleopDriveCommand.h"
#include "RobotMap.h"

TeleopDriveCommand::TeleopDriveCommand(Drivetrain & drivetrain, frc::Joystick & driveJoystick)
 : m_drivetrain(drivetrain), m_driveJoystick(driveJoystick)
{
    AddRequirements({&m_drivetrain});
}

void TeleopDriveCommand::Execute() 
{
    m_drivetrain.CurvatureDrive(
        m_conditioning.Condition(-m_driveJoystick.GetY()),
        m_conditioning.Condition(m_driveJoystick.GetTwist()),
        m_driveJoystick.GetRawButton(kQuickTurn));
}
