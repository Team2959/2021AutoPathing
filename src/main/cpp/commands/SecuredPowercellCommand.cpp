// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SecuredPowercellCommand.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
SecuredPowercellCommand::SecuredPowercellCommand(Intake& intake) : m_intake(intake) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SecuredPowercellCommand::Initialize()
{
    if(m_intake.IsIntakeRunning())
    {
        m_intake.SetConveyorSpeed(0);
        m_intake.SetIntakeSpeed(m_intake.GetIntakeFullSpeed());
        m_intake.IncrementPowercellCount();

        if(m_intake.GetPowercellCount() == 5)
        {
            m_intake.TravelingInit();
        }
    }
}
