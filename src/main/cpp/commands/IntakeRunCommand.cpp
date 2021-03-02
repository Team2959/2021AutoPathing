// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeRunCommand.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
IntakeRunCommand::IntakeRunCommand(Intake& intake, bool on) : m_intake(intake), m_turnOn(on) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void IntakeRunCommand::Initialize() 
{
  if(m_turnOn)
  {
    m_intake.LoadingInit();
  }
  else
  {
    m_intake.TravelingInit();
    m_intake.LeftBallFlipper(Intake::FeedingCylinderDirection::Opened);
    m_intake.RightBallFlipper(Intake::FeedingCylinderDirection::Opened);
  }
  
}
