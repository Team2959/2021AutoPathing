/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

// #include <experimental/filesystem>
#include <dirent.h>
#include <thread>

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

// namespace fs = std::experimental::filesystem;

void Robot::RobotInit() {
  m_container.RobotInit();
  m_autoChooser.SetName("Special Auto Modes");
  m_autoChooser.AddOption("Bounce", "Bounce");

  std::string paths_dir = "/home/lvuser/deploy/paths";

  DIR *dir;
  struct dirent *ent;
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

  m_autoChooser.SetName("Auto Mode");
  // for (auto& entry : fs::directory_iterator(paths_dir)) {
  //     m_autoChooser.AddOption(entry.path().filename().string(), entry.path().filename().string());
  // }
  frc::SmartDashboard::PutData(&m_autoChooser); 

  frc::SmartDashboard::PutBoolean("Curvature Drive", true);
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
