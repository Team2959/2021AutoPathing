/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/Command.h>
#include <subsystems/Drivetrain.h>
#include <subsystems/Intake.h>
#include "OI.h"
#include "commands/TeleopDriveCommand.h"

#include <wpi/SmallString.h>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer
{
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  frc2::Command* GetPathingCommand(wpi::SmallString<64> name);

  void RobotInit();
 private:
  // The robot's subsystems and commands are defined here...
  Drivetrain m_drivetrain;
  Intake m_intake;
  OI m_oi;
  
  frc2::Trigger m_newPowercellTrigger;
  frc2::Trigger m_securedPowercellTrigger;
  frc2::Trigger m_kickerTrigger;
  frc2::Trigger m_intakeOnTrigger;

  TeleopDriveCommand m_defaultDriveCommand {m_drivetrain, m_oi.m_driverJoystick};

  void ConfigureButtonBindings();
};


bool FileExists(std::string filename);