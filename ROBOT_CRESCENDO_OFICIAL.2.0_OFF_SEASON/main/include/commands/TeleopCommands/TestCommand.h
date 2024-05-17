// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/TopSubsystemGroup.h"
// #include "subsystems/TopSubsystems/ShooterSubsystem.h"
// #include "subsystems/TopSubsystems/GiraffeSubsystem.h"
// #include "subsystems/TopSubsystems/IntakeSubsystem.h"
// #include "subsystems/LimeLightBackSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TestCommand
    : public frc2::CommandHelper<frc2::Command, TestCommand> {
 public:
  TestCommand(TopSubsystemGroup& top_SubsystemParam);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  TopSubsystemGroup*top_Subsystem;
//    IntakeSubsystem*intake_Subsystem;
//  ShooterSubsystem*shooter_Subsystem;
//  GiraffeSubsystem*giraffe_Subsystem;
//  LimeLightBackSubsystem*lime_Subsystem;
};
