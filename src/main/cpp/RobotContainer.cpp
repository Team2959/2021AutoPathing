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
#include "commands/IntakeRunCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>

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
  frc::SmartDashboard::PutBoolean(kIntakeOnName, false);
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
  auto zeta = frc::SmartDashboard::GetNumber("ZETA", Drive::kRamseteZeta);
  auto b = frc::SmartDashboard::GetNumber("Ramsette B", Drive::kRamseteB);

  frc2::RamseteCommand ramseteCommand(
      exampleTrajectory, [this]() { return m_drivetrain.GetPose(); },
      frc::RamseteController(b,zeta),
      m_drivetrain.kDriveKinematics,
      [this](auto left, auto right) { m_drivetrain.CalculateOutput(left, right); },
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
  auto ramseteCommand = RamseteCommandFromPathWeaverJson(name, true);
  bool turnOn = frc::SmartDashboard::GetBoolean(kIntakeOnName, false);

  return new frc2::SequentialCommandGroup(
    IntakeRunCommand(m_intake, turnOn),
    std::move(ramseteCommand),
    frc2::InstantCommand([this] { m_drivetrain.SetVolts(0_V, 0_V); }),
    IntakeRunCommand(m_intake, false)
  );
}

frc2::Command* RobotContainer::BouncePathAuto()
{
  wpi::SmallString<64> name {"Bounce1.wpilib.json"};
  auto bounce1 = RamseteCommandFromPathWeaverJson(name, true);
  name = "Bounce2.wpilib.json";
  auto bounce2 = RamseteCommandFromPathWeaverJson(name);
  name = "Bounce3.wpilib.json";
  auto bounce3 = RamseteCommandFromPathWeaverJson(name);
  name = "Bounce4.wpilib.json";
  auto bounce4 = RamseteCommandFromPathWeaverJson(name);

  return new frc2::SequentialCommandGroup(
    std::move(bounce1),
    std::move(bounce2),
    std::move(bounce3),
    std::move(bounce4),
    frc2::InstantCommand([this] { m_drivetrain.SetVolts(0_V, 0_V); }));
}

frc2::RamseteCommand RobotContainer::RamseteCommandFromPathWeaverJson(wpi::SmallString<64> name, bool resetOdometry)
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

  if (resetOdometry)
  {
    m_drivetrain.ResetOdometry(trajectory.InitialPose());
  }

  auto zeta = frc::SmartDashboard::GetNumber("ZETA", Drive::kRamseteZeta);
  auto b = frc::SmartDashboard::GetNumber("Ramsette B", Drive::kRamseteB);

  frc2::RamseteCommand ramseteCommand(
      trajectory, [this]() { return m_drivetrain.GetPose(); },
      frc::RamseteController(b, zeta),
      m_drivetrain.kDriveKinematics,
      [this](auto left, auto right) { m_drivetrain.CalculateOutput(left, right); },
      {&m_drivetrain});
  return ramseteCommand;
}

std::string RobotContainer::GetLoggingData()
{
    return m_drivetrain.GetLoggingData();
}