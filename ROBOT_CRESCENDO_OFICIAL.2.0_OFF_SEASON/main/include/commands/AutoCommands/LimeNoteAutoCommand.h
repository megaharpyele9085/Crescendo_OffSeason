// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/LimeLightFrontSubsystem.h"
#include "subsystems/SwerveSubsystem.h"
#include "PIDController9085.h"
#include <frc/smartdashboard/SmartDashboard.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class LimeNoteAutoCommand
    : public frc2::CommandHelper<frc2::Command, LimeNoteAutoCommand> {
 public:
  LimeNoteAutoCommand(SwerveSubsystem*swerve_SubsystemParam,   LimeLightFrontSubsystem*limeFront_SubsystemParam);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  SwerveSubsystem* swerve_Subsystem;
  LimeLightFrontSubsystem* limeFront_Subsystem;
  frc2::PIDController9085 DriveYPid{0.012, 0, 0, 20, 0.39364_s, 0.01_s};
  frc2::PIDController9085 DriveRotPid{-0.007, 0, 0, 20, 0.39364_s, 0.01_s};
  frc2::PIDController9085 HeadingPID{-0.008, 0, 0, 20, 0.39364_s, 0.01_s};

  bool tavivoPt2 = false;
};
