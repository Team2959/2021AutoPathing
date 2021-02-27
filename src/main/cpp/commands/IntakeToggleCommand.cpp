// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeToggleCommand.h"

IntakeToggleCommand::IntakeToggleCommand(Intake& intake) : m_intake(intake) {}

// Called when the command is initially scheduled.
void IntakeToggleCommand::Initialize() 
{
  if (m_intake.IsIntakeRunning())
  {
      m_intake.TravelingInit(); //m_stateManager.StartState(States::Traveling);
      m_intake.LeftBallFlipper(Intake::FeedingCylinderDirection::Opened);
      m_intake.RightBallFlipper(Intake::FeedingCylinderDirection::Opened);
  }
  else
  {
      m_intake.LoadingInit(); // m_stateManager.StartState(States::Loading);
  }
}
