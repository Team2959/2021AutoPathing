// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ReverseConveyorCommand.h"

ReverseConveyorCommand::ReverseConveyorCommand(Intake& intake) : m_intake(intake) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ReverseConveyorCommand::Initialize() 
{
  m_intake.TravelingInit();
  m_intake.SetIntakeSpeed(-m_intake.GetIntakeFullSpeed());
  m_intake.SetConveyorSpeed(-m_intake.GetConveyorFullSpeedWhenLoading());;
}

// Called once the command ends or is interrupted.
void ReverseConveyorCommand::End(bool interrupted) 
{
  m_intake.TravelingInit();
}

