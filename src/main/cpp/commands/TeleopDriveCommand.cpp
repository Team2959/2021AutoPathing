#include "commands/TeleopDriveCommand.h"
#include "RobotMap.h"

TeleopDriveCommand::TeleopDriveCommand(Drivetrain & drivetrain, frc::Joystick & driveJoystick)
 : m_drivetrain(drivetrain), m_driveJoystick(driveJoystick)
{
    AddRequirements({&m_drivetrain});
    m_conditioning.SetDeadband(.1);
    m_conditioning.SetExponent(2.5);
    m_conditioning.SetRange(0.0, 1.0);
    m_turnCondiditioning.SetDeadband(.1);
    m_turnCondiditioning.SetExponent(4);
    m_turnCondiditioning.SetRange(0.0, 1.0);
}

void TeleopDriveCommand::Execute() 
{
    m_drivetrain.CurvatureDrive(
        m_conditioning.Condition(-m_driveJoystick.GetY()),
        m_turnCondiditioning.Condition(m_driveJoystick.GetTwist()),
        m_driveJoystick.GetRawButton(kQuickTurn));
    m_drivetrain.Periodic();
}
