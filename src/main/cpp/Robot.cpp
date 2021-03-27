/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>
#include <frc/Filesystem.h>

// #include <experimental/filesystem>
#include <dirent.h>
#include <thread>
#include <iostream>
#include <sstream>

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

// namespace fs = std::experimental::filesystem;

void Robot::RobotInit() {
  m_container.RobotInit();
  m_autoChooser.SetName("Auto Mode");
  m_autoChooser.AddOption("Bounce", "Bounce");
  m_autoChooser.AddOption("FixedGalacticRed", "FixedGalacticRed");
  m_autoChooser.AddOption("FixedGalacticBlue", "FixedGalacticBlue");

  std::string paths_dir = "/home/lvuser/deploy/paths";
  std::string raw_path_dir = "/home/lvuser/deploy/raw_paths";
  std::string raw_path_ending = ".path";

  frc::TrajectoryConfig config(Drive::kMaxSpeed, Drive::kMaxAcceleration);
  config.SetStartVelocity(0_mps);
  config.SetEndVelocity(0_mps);
  
  frc::CentripetalAccelerationConstraint constraint(Drive::kMaxCentripetalAcceleration);

  // Centripetal Acceleration (ac) = v^2/r
  // example v = 1m/s, r = 30" = 0.762m
  // a = 1 / 0.762 ~= 1.31 m/s^2
  //
  // find out the tightest turn we need to make and then figure out how fast we can make that without spinning out.
  // with that as the limit, we should be able to make all larger radius turns without skidding as well.

  config.AddConstraint(constraint);

  wpi::SmallString<128> deployDirectory;
  frc::filesystem::GetDeployDirectory(deployDirectory);
  //wpi::sys::path::append(deployDirectory, "paths");

  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir(raw_path_dir.c_str())) != NULL) {
    while ((ent = readdir(dir)) != NULL) {
      if(ent->d_type == DT_REG) {
        std::string name = ent->d_name;
        // Check that file ends with .path
        
        if (name.length() > raw_path_ending.length() && name.compare(name.length() - raw_path_ending.length(), raw_path_ending.length(), raw_path_ending) == 0) {
          
          // Read path way points into trajectory
          // https://github.com/wpilibsuite/PathWeaver/blob/bbba553201b24fff8e23509b0af7104f0bde3a35/src/main/java/edu/wpi/first/pathweaver/spline/wpilib/WpilibSpline.java
          // https://github.com/wpilibsuite/PathWeaver/blob/bbba553201b24fff8e23509b0af7104f0bde3a35/src/main/java/edu/wpi/first/pathweaver/Waypoint.java#L25
          
          wpi::SmallString<128> csv_path(deployDirectory);
          wpi::sys::path::append(csv_path, "raw_paths");
          wpi::sys::path::append(csv_path, name);
          std::ifstream csv_file;
          csv_file.open(csv_path.c_str());

          if (!csv_file.good() || !csv_file.is_open()) {
            std::cout << "Failed to open: " << csv_path.c_str() << std::endl;
            continue;
          }
          
          std::string line;
          std::getline(csv_file, line); // First header row

          std::vector<frc::Spline<5>::ControlVector> controlVectors;
          bool reversed = false;
          while (std::getline(csv_file, line)) {
            std::stringstream iss(line);
            std::string cell;

            std::getline(iss, cell, ',');
            double x = std::stod(cell);
            std::getline(iss, cell, ',');
            double y = std::stod(cell);
            std::getline(iss, cell, ',');
            double xtangent = std::stod(cell);
            std::getline(iss, cell, ',');
            double ytangent = std::stod(cell);
            std::getline(iss, cell, ','); // Fixed Theta
            std::getline(iss, cell, ',');
            reversed = reversed || (cell == "true");

            controlVectors.push_back({std::array<double, 3>{x, xtangent, 0}, std::array<double, 3>{y,ytangent,0}});
          }
          config.SetReversed(reversed);
          frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(controlVectors, config);
          wpi::SmallString<128> json_path(deployDirectory);
          wpi::sys::path::append(json_path, "paths");
          wpi::sys::path::append(json_path, (name.substr(0, name.length() - raw_path_ending.length()) + ".wpilib.json"));
          remove(json_path.c_str());
          frc::TrajectoryUtil::ToPathweaverJson(trajectory, json_path);
        }        
      }
    }
    closedir(dir);
  }

  if ((dir = opendir(paths_dir.c_str())) != NULL) {
    while ((ent = readdir(dir)) != NULL) {
      if(ent->d_type == DT_REG) {
        std::string name = ent->d_name;
        name = name.substr(0, name.length() - std::string(".wpilib.json").length());
        m_autoChooser.AddOption(name, ent->d_name);
      }
    }
    closedir(dir);
  }

  // for (auto& entry : fs::directory_iterator(paths_dir)) {
  //     m_autoChooser.AddOption(entry.path().filename().string(), entry.path().filename().string());
  // }
  frc::SmartDashboard::PutData(&m_autoChooser); 

  frc::SmartDashboard::PutBoolean("Curvature Drive", true);
  frc::SmartDashboard::PutNumber("Auto Max Velocity", Drive::kMaxSpeed.to<double>());
  frc::SmartDashboard::PutNumber("Auto Max Acceleration", Drive::kMaxAcceleration.to<double>());
  frc::SmartDashboard::PutNumber("Auto Max Centripetal Acceleration", Drive::kMaxCentripetalAcceleration.to<double>());
  //StartNewLogFile();
  frc::SmartDashboard::PutNumber("Drive/Deadband",.1);
  frc::SmartDashboard::PutNumber("Drive/Exponent",2.5);
  frc::SmartDashboard::PutNumber("Drive/Output Min",0.0);
  frc::SmartDashboard::PutNumber("Drive/Turn Deadband",.1);
  frc::SmartDashboard::PutNumber("Drive/Turn Exponent",4);
  frc::SmartDashboard::PutNumber("Drive/Turn Output Min",0.0);
  frc::SmartDashboard::PutNumber("Drive/Output Max", 1.0);
  frc::SmartDashboard::PutNumber("Drive/Turn Output Max", 1.0);
  frc::SmartDashboard::PutString("Odometry File", "");
  frc::SmartDashboard::PutNumber("ZETA", Drive::kRamseteZeta);
  frc::SmartDashboard::PutNumber("Ramsette B", Drive::kRamseteB);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() { 
  frc2::CommandScheduler::GetInstance().Run(); 
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() 
{
    if(m_logFile.is_open())
      m_logFile.close();
}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  StartNewLogFile();
  std::string file = m_autoChooser.GetSelected();
  if(file == "Bounce")
  {
      m_autonomousCommand = m_container.BouncePathAuto();
  }
  else if(file == "FixedGalacticRed")
  {
    if (m_Count % 2 == 0)
    {
      file = "GalacticSearchA_Red.wpilib.json";
    }
    else
    {
      file = "GalacticSearchB_Red.wpilib.json";
    }

      wpi::Twine twine{file};
      wpi::SmallString<64> smallString;
      twine.toVector(smallString);
      std::cout << smallString << std::endl;
      m_autonomousCommand = m_container.GetPathingCommand(smallString);
      m_Count++;
  }
  else if(file == "FixedGalacticBlue")
  {
    if (m_Count % 2 == 0)
    {
      file = "GalacticSearchA_Blue.wpilib.json";
    }
    else
    {
      file = "GalacticSearchB_Blue.wpilib.json";
    }

      wpi::Twine twine{file};
      wpi::SmallString<64> smallString;
      twine.toVector(smallString);
      std::cout << smallString << std::endl;
      m_autonomousCommand = m_container.GetPathingCommand(smallString);
      m_Count++;
  }
  else
  {
      wpi::Twine twine{file};
      wpi::SmallString<64> smallString;
      twine.toVector(smallString);
      std::cout << smallString << std::endl;
      m_autonomousCommand = m_container.GetPathingCommand(smallString);
  }
  if(m_autonomousCommand == nullptr)
  {
    m_autonomousCommand = m_container.GetAutonomousCommand();
  }

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {
  Log();
}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
  StartNewLogFile();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() 
{
    Log();
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

void Robot::StartNewLogFile()
{
    if(m_logFile.is_open()) 
        m_logFile.close();
    std::string usbDirectory = GetFirstDirectory("/media");
    std::string filename;
    if(usbDirectory.length() == 0)
    {
        std::string odometryDirectory = "/home/lvuser/odometry";
        filename = GetFirstModifiedFile(odometryDirectory);
        filename = odometryDirectory + "/" + filename;
    }
    else
    {
        usbDirectory = "/media/" + usbDirectory;
        int i;
        std::string odometryPrefix = m_autoChooser.GetSelected();
        std::string random_str = RandomString();
        for(i = 0; std::ifstream{usbDirectory + "/" + odometryPrefix + random_str + std::to_string(i) + ".csv"}.good(); i++);
        filename = usbDirectory + "/" + odometryPrefix + random_str + std::to_string(i) + ".csv";
    }
    std::cout << "Log File:" << filename << std::endl;
    m_logFile.open(filename);
    if(!m_logFile.good() || !m_logFile.is_open())
    {
        std::cout << "Log File failed to open" << std::endl;
    }
    m_logFile << "timestamp,angle,left,right,rotation,positionX,positionY,leftVelocity,rightVelocity,leftSetpoint,rightSetpoint\n" << std::flush;
    frc::SmartDashboard::PutString("Odometry File", filename);
}

void Robot::Log()
{
    std::string data = m_container.GetLoggingData();
    m_logFile << data << std::flush;
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
