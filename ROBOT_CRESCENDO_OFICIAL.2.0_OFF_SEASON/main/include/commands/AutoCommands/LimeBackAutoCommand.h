// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/LimeLightBackSubsystem.h"
#include "subsystems/SwerveSubsystem.h"
#include "PIDController9085.h"
#include <frc/Timer.h>
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class LimeBackAutoCommand
    : public frc2::CommandHelper<frc2::Command, LimeBackAutoCommand> {
 public:
  LimeBackAutoCommand(SwerveSubsystem*swerve_SubsystemParam, LimeLightBackSubsystem*limeLightBack_SubsystemParam, double TyValueParam, double TxValueParam);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  frc::Timer TimerToEnd{};

  SwerveSubsystem*swerve_Subsystem;
  LimeLightBackSubsystem*limeLightBack_Subsystem;
  double TyValue;
  double TxValue;
  double LimeTyPid_P = -0.01;
  double LimeTyPid_I = 0.0;
  double LimeTyPid_D = 0.0;
  frc2::PIDController9085 SpeedTyPID{LimeTyPid_P, LimeTyPid_I, LimeTyPid_D, 20, 0.39364_s, 0.01_s};
  double LimeTxPid_P = -0.01;
  double LimeTxPid_I = 0.0;
  double LimeTxPid_D = 0.0;
  frc2::PIDController9085 SpeedTxPID{LimeTxPid_P, LimeTxPid_I, LimeTxPid_D, 20, 0.39364_s, 0.01_s};

  double SpeedYChassis;
  double SpeedXChassis;
};
