// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include "subsystems/SwerveSubsystem.h"
#include "PIDController9085.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoYTimerCommand
    : public frc2::CommandHelper<frc2::Command, AutoYTimerCommand> {
 public:
  AutoYTimerCommand(units::second_t secondsParam, SwerveSubsystem*swerve_SubsystemParam, bool isInverted);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  frc::Timer TimeToGoY{};

  SwerveSubsystem*swerve_Subsystem;

  bool isYInverted;

  units::second_t secondsY;

  frc2::PIDController9085 HeadingPID{-0.026, 0, 0, 20, 0.39364_s, 0.01_s};
};
