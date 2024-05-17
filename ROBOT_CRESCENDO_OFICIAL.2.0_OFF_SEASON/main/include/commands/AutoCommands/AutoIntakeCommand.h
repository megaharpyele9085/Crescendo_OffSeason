// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/TopSubsystems/IntakeSubsystem.h"
#include <frc/Timer.h>
#include "subsystems/SwerveSubsystem.h"
#include "PIDController9085.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoIntakeCommand
    : public frc2::CommandHelper<frc2::Command, AutoIntakeCommand> {
 public:
  AutoIntakeCommand(IntakeSubsystem*intake_SubsystemParam, SwerveSubsystem*swerve_SubsystemParam);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  IntakeSubsystem*intake_Subsystem;

  SwerveSubsystem*swerve_Subsystem;

  frc::Timer TimeToCatch{};

  frc2::PIDController9085 HeadingPID{-0.015, 0, 0, 20, 0.39364_s, 0.01_s};
};
