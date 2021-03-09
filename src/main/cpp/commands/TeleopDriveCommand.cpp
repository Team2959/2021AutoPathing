#include "commands/TeleopDriveCommand.h"
#include "RobotMap.h"
#include <frc/smartdashboard/SmartDashboard.h>

TeleopDriveCommand::TeleopDriveCommand(Drivetrain & drivetrain, frc::Joystick & driveJoystick, frc::Joystick& leftTankDriveJoystick)
 : m_drivetrain(drivetrain), m_driveJoystick(driveJoystick), m_leftTankDriveJoystick(leftTankDriveJoystick)
{
    AddRequirements({&m_drivetrain});
    // m_conditioning.SetDeadband(kDefaultDeadband);
    // m_conditioning.SetExponent(kDefaultExponent);
    // m_conditioning.SetRange(kDefaultOutputOffset, 1.0);
    // m_turnCondiditioning.SetDeadband(kDefaultDeadband);
    // m_turnCondiditioning.SetExponent(kDefaultExponent);
    // m_turnCondiditioning.SetRange(kDefaultOutputOffset, 1.0);
    m_conditioning.SetDeadband(.1);
    m_conditioning.SetExponent(2.5);
    m_conditioning.SetRange(0.0, 1.0);
    m_turnCondiditioning.SetDeadband(.1);
    m_turnCondiditioning.SetExponent(4);
    m_turnCondiditioning.SetRange(0.0, 1.0);

}

void TeleopDriveCommand::Initialize()
{
    m_usingCurvatureDrive = frc::SmartDashboard::GetBoolean("Curvature Drive", true);
    m_conditioning.SetDeadband(frc::SmartDashboard::GetNumber("Drive/Deadband",.1));
    m_conditioning.SetExponent(frc::SmartDashboard::GetNumber("Drive/Exponent",2.5));
    m_turnCondiditioning.SetRange(frc::SmartDashboard::GetNumber("Drive/Output Min",0.0), frc::SmartDashboard::GetNumber("Drive/Output Max", 1.0));
    m_turnCondiditioning.SetDeadband(frc::SmartDashboard::GetNumber("Drive/Turn Deadband",.1));
    m_turnCondiditioning.SetExponent(frc::SmartDashboard::GetNumber("Drive/Turn Exponent",4));
    m_turnCondiditioning.SetRange(frc::SmartDashboard::GetNumber("Drive/Turn Output Min",0.0), frc::SmartDashboard::GetNumber("Drive/Turn Output Max", 1.0));
}

void TeleopDriveCommand::Execute() 
{
    if(m_usingCurvatureDrive)
    {
        m_drivetrain.CurvatureDrive(
            -m_conditioning.Condition(m_driveJoystick.GetY()),
            m_turnCondiditioning.Condition(m_driveJoystick.GetTwist()),
            m_driveJoystick.GetRawButton(kQuickTurn));
    }
    else
    {
        m_drivetrain.TankDrive(
            -m_conditioning.Condition(m_leftTankDriveJoystick.GetY()),
            -m_conditioning.Condition(m_driveJoystick.GetY())
        );
    }
    m_drivetrain.Periodic();
}
