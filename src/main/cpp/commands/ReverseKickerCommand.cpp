// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ReverseKickerCommand.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
ReverseKickerCommand::ReverseKickerCommand(Intake& intake) : m_intake(intake) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ReverseKickerCommand::Initialize() 
{
  m_intake.TravelingInit();
  m_intake.SetIntakeSpeed(-m_intake.GetIntakeFullSpeed());
  m_intake.SetConveyorSpeed(-m_intake.GetConveyorFullSpeedWhenLoading());
  m_intake.SetKickerSpeed(-m_intake.GetKickerFullSpeed());
}

void ReverseKickerCommand::End(bool interrupted)
{
  m_intake.TravelingInit();
}
