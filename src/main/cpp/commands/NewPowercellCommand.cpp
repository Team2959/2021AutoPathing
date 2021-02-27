// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/NewPowercellCommand.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
NewPowercellCommand::NewPowercellCommand(Intake& intake) : m_intake(intake) {}

// Called when the command is initially scheduled.
void NewPowercellCommand::Initialize() 
{
    if(m_intake.IsIntakeRunning())
    {
        if(m_intake.GetPowercellCount() == 4)
        {
            m_intake.IncrementPowercellCount();
            m_intake.TravelingInit();
        }
        else
        {
            m_intake.SetIntakeSpeed(m_intake.GetIntakeFullSpeed() * 0.5);
            m_intake.SetConveyorSpeed(m_intake.GetConveyorFullSpeedWhenLoading());
        }
        
    }
}
