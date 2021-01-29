/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"
#include "commands/IntakeSpeedCommand.h"

#include <frc/controller/RamseteController.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc2/command/RamseteCommand.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>

RobotContainer::RobotContainer()
{
  // Initialize all of your commands and subsystems here
  m_drivetrain.SetDefaultCommand(m_defaultDriveCommand);

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings()
{
  // Configure your button bindings here
  m_oi.m_intakeSpeedButton.WhenHeld(IntakeSpeedCommand(m_intake, 0.75));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return nullptr;
}

frc2::Command* RobotContainer::GetPathingCommand(wpi::SmallString<64> name)
{
    wpi::SmallString<64> deployDirectory;
    frc::filesystem::GetDeployDirectory(deployDirectory);
    wpi::sys::path::append(deployDirectory, "paths");
    wpi::sys::path::append(deployDirectory, name);

    frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);

  frc2::RamseteCommand* ramseteCommand = new frc2::RamseteCommand(
      trajectory, [this]() { return m_drivetrain.GetPose(); },
      frc::RamseteController(Drive::kRamseteB,
                             Drive::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(
          Drive::ks, Drive::kv, Drive::ka),
      m_drivetrain.kDriveKinematics,
      [this] { return m_drivetrain.GetWheelSpeeds(); },
      frc2::PIDController(Drive::kPDriveVel, 0, 0),
      frc2::PIDController(Drive::kPDriveVel, 0, 0),
      [this](auto left, auto right) { m_drivetrain.SetVolts(left, right); },
      {&m_drivetrain});
  return ramseteCommand;
}
