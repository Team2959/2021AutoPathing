// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

#include "subsystems/Intake.h"

class ReverseIntakeCommand
    : public frc2::CommandHelper<frc2::CommandBase,
                                 ReverseIntakeCommand> {
 public:
  ReverseIntakeCommand(Intake& intake);

  void Initialize() override;
  void End(bool interrupted) override;
private:
  Intake& m_intake;
};
