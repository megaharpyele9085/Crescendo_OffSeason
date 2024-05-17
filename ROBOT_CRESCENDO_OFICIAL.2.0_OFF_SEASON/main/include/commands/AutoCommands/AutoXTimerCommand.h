// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include "subsystems/SwerveSubsystem.h"
#include "PIDController9085.h"
#include <frc/DriverStation.h>


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoXTimerCommand
    : public frc2::CommandHelper<frc2::Command, AutoXTimerCommand> {
 public:
  AutoXTimerCommand(units::second_t secondsParam, SwerveSubsystem*swerve_SubsystemParam, double veloParam, bool IsXInvertedParam);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  frc::Timer TimeToGoX{};

  double xVelo;

  bool IsXInverted;

  SwerveSubsystem*swerve_Subsystem;

  frc2::PIDController9085 HeadingPID{-0.026, 0, 0, 20, 0.39364_s, 0.01_s};

  units::second_t secondsX;
};
