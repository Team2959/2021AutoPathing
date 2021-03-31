// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/KickerPowercellCommand.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
KickerPowercellCommand::KickerPowercellCommand(Intake& intake) : m_intake(intake) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void KickerPowercellCommand::Initialize() 
{
    if(m_intake.IsIntakeRunning())
    {
        m_intake.SetKickerSpeed(0);
    }   
}
