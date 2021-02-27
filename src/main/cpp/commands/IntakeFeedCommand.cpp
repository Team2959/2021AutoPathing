// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeFeedCommand.h"

IntakeFeedCommand::IntakeFeedCommand(Intake& intake) : m_intake(intake) {}

// Called when the command is initially scheduled.
void IntakeFeedCommand::Initialize() 
{
    m_intake.LoadingInit();
}

// Called repeatedly when this Command is scheduled to run
void IntakeFeedCommand::Execute() 
{
    m_intake.ProcessStickySwitches();
    m_intake.Feed();
}

// Called once the command ends or is interrupted.
void IntakeFeedCommand::End(bool interrupted) 
{
    m_intake.TravelingInit();
}

