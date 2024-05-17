// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/TopSubsystems/ShooterSubsystem.h"
#include "subsystems/LimeLightBackSubsystem.h"
#include "subsystems/SwerveSubsystem.h"
#include "subsystems/TopSubsystems/GiraffeSubsystem.h"
#include "subsystems/TopSubsystems/IntakeSubsystem.h"
#include "PIDController9085.h"
#include "Constants.h"
#include <frc/Timer.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoShooterCommand
    : public frc2::CommandHelper<frc2::Command, AutoShooterCommand> {
 public:
  AutoShooterCommand(ShooterSubsystem*shooter_SubsystemParam, LimeLightBackSubsystem*limeLightBack_SubsystemParam, SwerveSubsystem*swerve_SubsystemParam, GiraffeSubsystem*Giraffe_SubsystemParam, IntakeSubsystem*intake_SubsystemParam);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  ShooterSubsystem*shooter_Subsystem;

  LimeLightBackSubsystem*limeLightBack_Subsystem;

  SwerveSubsystem*swerve_Subsystem;

  GiraffeSubsystem*Giraffe_Subsystem;

  IntakeSubsystem*intake_Subsystem;

  frc::Timer TimeToShoot{};

  private:
  double ShooterVelocity = 90;
  double rightIntakeVelocity = -0.7;
  double leftIntakeVelocity = 0.7;

  
  double LimeTxPid_P = -0.012;
  double LimeTxPid_I = 0.0;
  double LimeTxPid_D = 0.0;
  frc2::PIDController9085 LimeLightPid_Tx{LimeTxPid_P, LimeTxPid_I, LimeTxPid_D, 20, 0.39364_s, 0.01_s};
};
