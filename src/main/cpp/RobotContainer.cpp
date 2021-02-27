/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"

#include <frc/controller/RamseteController.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc2/command/RamseteCommand.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <fstream>
#include "commands/ReverseConveyorCommand.h"
#include "commands/ReverseIntakeCommand.h"
#include "commands/ReverseKickerCommand.h"
#include "commands/IntakeToggleCommand.h"
#include "commands/NewPowercellCommand.h"
#include "commands/SecuredPowercellCommand.h"
#include "commands/KickerPowercellCommand.h"
#include "commands/IntakeFeedCommand.h"

bool FileExists(std::string filename)
{
  std::fstream check;
  check.open(filename);
  return check.is_open();
}

RobotContainer::RobotContainer()
  : m_newPowercellTrigger([this](){ return m_intake.GetSensorPressed(Intake::SensorLocation::NewPowercell); }),
    m_securedPowercellTrigger([this](){ return m_intake.GetSensorPressed(Intake::SensorLocation::SecuredPowercell); }),
    m_kickerTrigger([this](){ return m_intake.GetSensor(Intake::SensorLocation::Kicker); }),
    m_intakeOnTrigger([this](){ return m_intake.IsIntakeRunning(); })
{
  // Initialize all of your commands and subsystems here
  m_drivetrain.SetDefaultCommand(m_defaultDriveCommand);

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::RobotInit()
{
  m_intake.OnRobotInit();
}

void RobotContainer::ConfigureButtonBindings()
{
  // Configure your button bindings here
  m_oi.m_reverseConveyorButton.WhenHeld(ReverseConveyorCommand(m_intake));
  m_oi.m_reverseIntakeButton.WhenHeld(ReverseIntakeCommand(m_intake));
  m_oi.m_reverseKickerButton.WhenHeld(ReverseKickerCommand(m_intake));
  m_oi.m_intakeToggleButton.WhenHeld(IntakeToggleCommand(m_intake));
  m_newPowercellTrigger.WhenActive(NewPowercellCommand(m_intake));
  m_securedPowercellTrigger.WhenActive(SecuredPowercellCommand(m_intake));
  m_kickerTrigger.WhenInactive(KickerPowercellCommand(m_intake));
  m_intakeOnTrigger.WhileActiveOnce(IntakeFeedCommand(m_intake));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Create a voltage constraint to ensure we don't accelerate too fast
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
      frc::SimpleMotorFeedforward<units::meters>(
          Drive::ks, Drive::kv, Drive::ka),
      m_drivetrain.kDriveKinematics, 10_V);

  // Set up config for trajectory
  frc::TrajectoryConfig config(Drive::kMaxSpeed,
                               Drive::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drivetrain.kDriveKinematics);
  // Apply the voltage constraint
  config.AddConstraint(autoVoltageConstraint);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass the config
      config);

  frc2::RamseteCommand ramseteCommand(
      exampleTrajectory, [this]() { return m_drivetrain.GetPose(); },
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

  // Reset odometry to the starting pose of the trajectory.
  m_drivetrain.ResetOdometry(exampleTrajectory.InitialPose());

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(ramseteCommand),
      frc2::InstantCommand([this] { m_drivetrain.SetVolts(0_V, 0_V); }, {}));
}

frc2::Command* RobotContainer::GetPathingCommand(wpi::SmallString<64> name)
{
  wpi::SmallString<128> deployDirectory;
  frc::filesystem::GetDeployDirectory(deployDirectory);
  wpi::sys::path::append(deployDirectory, "paths");
  wpi::sys::path::append(deployDirectory, name);
  std::cout << deployDirectory << std::endl;
  // if(!FileExists(std::string(deployDirectory.c_str())))
  // {
  //   return nullptr;
  // }
  frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);  

  frc2::RamseteCommand ramseteCommand = frc2::RamseteCommand(
      trajectory, [this]() { return m_drivetrain.GetPose(); },
      frc::RamseteController(Drive::kRamseteB,
                             Drive::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(
          Drive::ks, Drive::kv, Drive::ka),
      m_drivetrain.kDriveKinematics,
      [this] { return m_drivetrain.GetWheelSpeeds(); },
      frc2::PIDController(Drive::kPDriveVel, Drive::kI, 0),
      frc2::PIDController(Drive::kPDriveVel, Drive::kI, 0),
      [this](auto left, auto right) { m_drivetrain.SetVolts(left, right); },
      {&m_drivetrain});

  m_drivetrain.ResetOdometry(trajectory.InitialPose());

  return new frc2::SequentialCommandGroup(
    std::move(ramseteCommand),
    frc2::InstantCommand([this] { m_drivetrain.SetVolts(0_V, 0_V); })
  );
}
